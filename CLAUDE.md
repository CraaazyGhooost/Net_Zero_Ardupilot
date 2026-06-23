# Net Zero ArduPilot — Multi-Drone Parallel Flight System

This project is a fork of ArduPilot that implements physically-connected multi-drone parallel flight. One "master" drone controls one or more "slave" drones via MAVLink-over-UART, replicating arm/disarm signals and motor control outputs.

## Architecture Overview

```
[RC Controller] --> [Master Drone (SYSID=1)] --UART/MAVLink--> [Slave Drone (SYSID≠1)]

Master computes its own motor outputs from RC input, then sends them to slaves via
custom MAVLink messages. Slaves receive these and apply them directly to their motors.
Arm/disarm is also synchronized from master to slaves.
```

## Key Files and Their Roles

### Custom MAVLink Messages
- **`modules/mavlink/message_definitions/v1.0/ardupilotmega.xml`** (lines ~2091-2103):
  Defines two custom messages:
  - `NET_ZERO_MAVLINK` (ID `17001`): Multi-purpose control message with `uint8 test1, test2`
    - `(1, 0)` — Master discovery probe (router handshake)
    - `(2, 0)` — Slave acknowledgment response
    - `(10, 10)` — Arm command
    - `(10, 20)` — Disarm command
  - `SUB_DRONE_CONTROL` (ID `17002`): Motor PWM replication — `uint8 target, uint16 motor1-4`

### Net Zero Protocol (Topology Router)
- **`libraries/AP_NetZero/AP_NetZero.h`** / **`libraries/AP_NetZero/AP_NetZero.cpp`**:
  - `connection` struct: Tracks one MAVLink link (direction, target SYSID, UART serial ID, MAVLink channel)
  - `NetZeroRouter` class: Maintains one `father` connection + up to `son_max` (default 2) slave connections
  - `SubDroneCache[10][4]`: Global buffer storing motor PWM values from master, indexed by target SYSID
  - `get_mavlink_chan_by_uart()`: Resolves a UART number to its MAVLink channel
  - `NetZeroRouter::set_father_uart()`: Configures the "upstream" connection
  - `NetZeroRouter::add_son_uart()`: Registers a slave after successful handshake
  - Backups: `backup_son[]` = `{4, 0xFF, 5, 0xFF}`, `backup_dir[]` = `{front, left, back, right}` — UARTs to probe for new slaves

### Router Discovery (Master Side)
- **`ArduCopter/Copter.cpp`** `update_router()` (line 269): Runs at 10Hz on master (SYSID=1 only). Iterates `backup_son[]` and sends `NET_ZERO_MAVLINK(1, 0)` as discovery probe on each valid backup UART.
- **`ArduCopter/wscript`**: Adds `AP_NetZero` to the Copter library list so the router implementation is linked into the vehicle build.

### Message Handlers (Both Master and Slave)
- **`ArduCopter/GCS_MAVLink_Copter.cpp`**:
  - `handle_message()` (line 1270): Dispatches custom message IDs to handlers
  - `handle_message_net_zero_command()` (line 1211):
    - Case `test1=1`: Slave receives discovery, sends back `(2, 0)` ack, switches to `SLAVE` mode
    - Case `test1=2`: Master receives slave ack, calls `net_zero_router.add_son_uart()` to register
    - Case `test1=10`: Arm (`test2=10`) or disarm (`test2=20`) relay from master — slave calls `AP::arming().arm()`/`disarm()`
  - `handle_message_sub_drone_control()` (line 1260): Slave receives motor PWM, stores into `SubDroneCache[target]`

### Motor Output Replication
- **`libraries/AP_Motors/AP_MotorsMatrix.cpp`**:
  - After computing `_actuator[]` outputs (line 180):
    - Master sends PWM to all connected sons via `mavlink_msg_sub_drone_control_send()` (line 191)
    - If current mode is SLAVE (mode 31): reads motor outputs from `SubDroneCache` instead of using self-computed values (line 210)

### Arm/Disarm Synchronization
- **`ArduCopter/AP_Arming_Copter.cpp`**:
  - `send_net_zero_arm_state(arm)` (line 8): Sends `NET_ZERO_MAVLINK(10, arm?10:20)` to all registered sons. Falls back to broadcasting on channels 1-3 if no sons are registered yet.
  - Called at end of `arm()` success (line 810) and `disarm()` success (line 883).

### MAVLink Infrastructure Changes
- **`libraries/GCS_MAVLink/GCS.h`** (line 394-395): Declares `send_net_zero_mavlink()` and `send_sub_drone_control()` on base `GCS_MAVLINK`
- **`libraries/GCS_MAVLink/GCS_Common.cpp`**:
  - `mavlink_id_to_ap_message_id_map[]` (line 1206-1207): Maps custom message IDs to `MSG_NET_ZERO_MAVLINK` / `MSG_SUB_DRONE_CONTROL`
  - `send_net_zero_mavlink()` / `send_sub_drone_control()` (line 6170-6176): Convenience senders
  - `try_send_message()` (line 6410-6413): Registers both message types in the stream (no periodic streaming, handled manually)
- **`ArduCopter/GCS_MAVLink_Copter.h`** (line 40-41): Declares `handle_message_net_zero_command()` and `handle_message_sub_drone_control()`

### Flight Modes
- **`ArduCopter/mode.h`**:
  - `ModeMaster` (mode 29): Master pilot mode — `run()` is currently empty (stub)
  - `ModeSlave` (mode 31): Slave mode — `run()` is currently empty (stub); the actual slave behavior is in `AP_MotorsMatrix.cpp` checking `mode_number == 31`
  - `ModeDocking` (mode 32): Docking maneuver mode — `run()` is currently empty (stub)
- Implementation files: `mode_master.cpp`, `mode_slave.cpp`, `mode_docking.cpp` (all stubs with commented-out Stabilize-like code)

### AP_Mission Integration (Commented Out)
- **`libraries/AP_Mission/AP_Mission.cpp`** (lines 1470-1473, 1997-2000): Commented-out support for `NET_ZERO_MAVLINK` as a mission command type. Not currently active.

## Data Flow Summary

```
1. DISCOVERY:
   Master update_router() --NET_ZERO_MAVLINK(1,0)--> Slave
   Slave handle_message_net_zero_command() --NET_ZERO_MAVLINK(2,0)--> Master
   Master registers slave via add_son_uart()

2. ARM/DISARM:
   Master arm()/disarm() --> send_net_zero_arm_state() --NET_ZERO_MAVLINK(10,10/20)--> Slave
   Slave receives --> AP::arming().arm()/disarm()

3. FLIGHT CONTROL (every motor output cycle):
   Master computes motors --> sends SUB_DRONE_CONTROL(motor1-4) to each son
   Slave receives --> stores in SubDroneCache
   Slave AP_MotorsMatrix::output_to_motors() --> reads SubDroneCache instead of self-computed
```

## Configuration

- Master must have `SYSID_THISMAV = 1`
- Slave must have `SYSID_THISMAV ≠ 1`
- Physical UART connections: UART4 (serial 4) and UART5 (serial 5) are the default backup UARTs for slave discovery
- The protocol is designed for a star topology: one master, multiple slaves connected via dedicated UARTs

## Important Notes

- Mode `MASTER`/`SLAVE`/`DOCKING` classes exist but their `run()` methods are stubs — actual slave control logic lives in `AP_MotorsMatrix::output_to_motors()` checking `mode_number == 31`
- The router's `backup_son[]` array is hardcoded with UART IDs {4, 0xFF, 5, 0xFF} — only UART4 and UART5 are probed
- `son_max = 2` limits the system to 2 slave drones currently
- The `SubDroneCache` is sized for 10 targets but only `son_max` (2) are actually used

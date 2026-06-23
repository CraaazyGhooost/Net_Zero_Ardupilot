# Net Zero Combination Position

## Goal

Each vehicle in the rigid Net Zero combination tracks an integer grid position in a Cartesian coordinate system centered on the master vehicle.

The master vehicle is always at `(0, 0)`. A directly connected slave gets its position from the physical connection direction used by the master during discovery:

| Direction | Position |
|---|---:|
| `front` | `(0, 1)` |
| `back` | `(0, -1)` |
| `left` | `(-1, 0)` |
| `right` | `(1, 0)` |

The coordinates are topological grid coordinates, not meters. They describe which rectangular vehicle cell the drone occupies in the rigid body.

## Implementation

The position state lives in `libraries/AP_NetZero` with the router state:

- `net_zero_position` stores `valid`, `x`, and `y`.
- `connection::position` stores the coordinate of a registered peer in the master-centered grid.
- `NetZeroRouter::self_position` stores the local vehicle's own coordinate.
- `net_zero_position_from_dir()` maps a physical connection direction to the integer coordinate.
- `net_zero_dir_to_protocol()` and `net_zero_dir_from_protocol()` encode the direction into the existing `NET_ZERO_MAVLINK.test2` byte.

This avoids adding a new MAVLink message or regenerating MAVLink headers. Existing message meanings remain compatible:

- `NET_ZERO_MAVLINK(test1=1, test2=direction)` is still the master discovery probe, but now also carries the physical side being probed.
- `NET_ZERO_MAVLINK(test1=2, test2=0)` remains the slave acknowledgement.
- `NET_ZERO_MAVLINK(test1=10, test2=10/20)` remains arm/disarm synchronization.

## Update Flow

1. During startup, `NetZeroRouter::init_position_from_sysid()` sets the master (`SYSID=1`) to `(0, 0)` and leaves other vehicles unknown until discovered.
2. The master `Copter::update_router()` probes each configured candidate UART and sends the matching `backup_dir[]` value in `test2`.
3. A slave receiving `test1=1` decodes `test2`, records its father connection, and updates `self_position` from the decoded direction.
4. The slave then replies with `test1=2`.
5. The master receives the acknowledgement and calls `add_son_uart()`, which stores the slave connection and the slave's coordinate in `connection::position`.

As the combination is assembled, a vehicle's coordinate changes at the moment it accepts a discovery probe for the side it is physically connected to. If a different physical connection is later accepted, the same setter updates `self_position` to the new coordinate.

## Current Scope

The current implementation models the topology already supported by the router: one master and up to two directly connected slaves, with candidate sides from `backup_dir[]`.

The design intentionally keeps the position logic in `AP_NetZero` so later work can extend it to multi-hop rigid structures. A future multi-hop version should transmit the parent coordinate and add the local direction offset, rather than assuming every slave is directly adjacent to the master.

## Future Design: Full Combination Shape

This section records the intended design for tracking the whole rigid combination shape. It is not implemented yet.

Each vehicle should eventually know the entire connected body, not only its own coordinate. The shape can be represented as a tree:

- Each drone is a node, identified by MAVLink `sysid`.
- Each rigid connection is an edge, represented by an extended `connection`.
- The master is the root node, normally `sysid=1`, fixed at `(0, 0)`.
- Every child coordinate is derived from its parent coordinate plus the edge direction offset.

The tree edge should carry:

- `parent_sysid`
- `child_sysid`
- `dir_from_parent`
- `parent_pos`
- `child_pos`
- `serial_id`
- `mavlink_chan`

Every vehicle would store a local copy of the topology in `AP_NetZero`, for example:

```cpp
struct net_zero_node {
    bool valid;
    uint8_t sysid;
    net_zero_position position;
};

struct net_zero_edge {
    bool valid;
    uint8_t parent_sysid;
    uint8_t child_sysid;
    direc dir_from_parent;
    connection link;
};
```

`NetZeroRouter` would own a fixed-capacity topology object containing `nodes[]`, `edges[]`, `root_sysid`, `self_sysid`, and a `topology_version`. Fixed arrays are preferred over dynamic allocation.

The master should be the authoritative topology owner. When a new edge is discovered, the master updates its topology, increments `topology_version`, then sends the full topology to all registered vehicles. Slaves replace their local topology only after receiving a complete version.

The existing `NET_ZERO_MAVLINK` message is too small for full-tree synchronization. A future implementation should add a dedicated MAVLink message, for example one edge per packet:

```text
NET_ZERO_TOPOLOGY_EDGE
  uint8 version
  uint8 edge_index
  uint8 edge_count
  uint8 parent_sysid
  uint8 child_sysid
  int8 parent_x
  int8 parent_y
  int8 child_x
  int8 child_y
  uint8 direction
```

Receivers would collect all `edge_count` packets for the same `version`, rebuild `nodes[]` and `edges[]`, then apply that topology atomically.

When adding an edge, the topology layer should reject invalid trees:

- `parent_sysid` must differ from `child_sysid`.
- The parent node must already exist.
- A child must not already have a different parent.
- A child coordinate must not collide with an existing different node.
- Adding the edge must not create a cycle.
- Node and edge counts must fit within fixed compile-time capacities.

The one-second debug output should eventually report topology summary as well as local position:

```text
Net zero topology: self=4 pos=(0,1) ver=3 nodes=3 edges=2
```

Detailed edge output can be added behind a debug option if needed:

```text
edge 0: 1(0,0) --front--> 4(0,1)
```

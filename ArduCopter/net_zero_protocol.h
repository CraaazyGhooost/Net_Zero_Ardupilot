#ifndef __NET_ZERO_PROTOCOL_H__
#define __NET_ZERO_PROTOCOL_H__

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <stdint.h>

enum direc{
    front, left, back, right,
    no_dir = 0xFF
};

class connection{
public:
    bool valid;
    direc dir;
    uint8_t target_id;
    uint8_t serial_id;
    uint8_t mavlink_chan; // the MAVLink channel associated with this connection, if valid
    connection() : valid(false), dir(front), target_id(0xFF), serial_id(0xFF), mavlink_chan(0xFF) {} // default constructor with invalid UART
    connection(bool v, direc d, uint8_t t_id, uint8_t s_id, uint8_t m_ch): valid(v), dir(d), target_id(t_id), serial_id(s_id), mavlink_chan(m_ch) {}
};


class NetZeroRouter {
public:
    NetZeroRouter();

    // param: uart = real uart number, e.g. 0 for SERIAL0, 1 for SERIAL1, etc.
    bool set_father_uart(direc d, uint8_t s_id, uint8_t t_id); // set father connection with a connection struct
    bool add_son_uart(direc d, uint8_t s_id, uint8_t t_id); // add a son connection with a connection struct, up to 8 sons

    connection get_father();
    connection get_son(uint8_t son);// param: son = 0-7 for son1-son8   ret: real uart number
    uint8_t get_son_count();


    // delayed UART enable: disable a UART at init, re-enable after delay
    void init_delayed_uart();        // call from Copter::init_ardupilot()
    void delayed_uart_tick();        // timer callback to check and re-enable

    connection father;
    connection son[4];
    uint8_t backup_son[4]; // backup serial id!!!
    direc backup_dir[4];
    uint8_t son_max;
    uint8_t son_count;

    // state for delayed UART enable
    uint8_t  _delayed_uart_id;
    uint32_t _delayed_uart_enable_ms;
    bool     _delayed_uart_pending;

};

extern NetZeroRouter net_zero_router; // global instance

extern uint16_t SubDroneCache[10][4]; // global cache for sub-drone data, 10 sets of 4 data points each

uint8_t get_mavlink_chan_by_uart(uint8_t uart_id); // helper function to get mavlink channel by uart id

// delayed UART enable: disable a UART at init time, re-enable after a delay
#define NETZERO_DELAYED_UART_ID       4       // SERIAL4 to delay
#define NETZERO_DELAYED_UART_ENABLE_MS 3000   // delay 3 seconds before re-enable

void net_zero_delayed_uart_init();  // call from Copter::init_ardupilot()

#endif
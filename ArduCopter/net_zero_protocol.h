#ifndef __NET_ZERO_PROTOCOL_H__
#define __NET_ZERO_PROTOCOL_H__

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
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
    connection() : valid(false), dir(front), target_id(0xFF), serial_id(0xFF) {} // default constructor with invalid UART
    connection(bool v, direc d, uint8_t t_id, uint8_t s_id): valid(v), dir(d), target_id(t_id), serial_id(s_id) {}
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


    connection father;
    connection son[4];
    uint8_t backup_son[4]; // backup serial id!!!
    direc backup_dir[4];
    uint8_t son_max;
    uint8_t son_count;


};

extern NetZeroRouter net_zero_router; // global instance

#endif
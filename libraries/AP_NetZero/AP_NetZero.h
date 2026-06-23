#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <stdint.h>

enum direc {
    front, left, back, right,
    no_dir = 0xFF
};

class net_zero_position {
public:
    bool valid;
    int8_t x;
    int8_t y;

    net_zero_position() : valid(false), x(0), y(0) {}
    net_zero_position(int8_t pos_x, int8_t pos_y, bool position_valid) :
        valid(position_valid),
        x(pos_x),
        y(pos_y) {}
};

class connection {
public:
    bool valid;
    direc dir;
    uint8_t target_id;
    uint8_t serial_id;
    uint8_t mavlink_chan;
    net_zero_position position;

    connection() :
        valid(false),
        dir(front),
        target_id(0xFF),
        serial_id(0xFF),
        mavlink_chan(0xFF),
        position() {}
    connection(bool v,
               direc d,
               uint8_t t_id,
               uint8_t s_id,
               uint8_t m_ch,
               const net_zero_position &pos = net_zero_position()) :
        valid(v),
        dir(d),
        target_id(t_id),
        serial_id(s_id),
        mavlink_chan(m_ch),
        position(pos) {}
};

class NetZeroRouter {
public:
    NetZeroRouter();

    bool set_father_uart(direc d, uint8_t s_id, uint8_t t_id);
    bool add_son_uart(direc d, uint8_t s_id, uint8_t t_id);

    connection get_father();
    connection get_son(uint8_t son);
    uint8_t get_son_count();
    net_zero_position get_self_position() const;
    void send_position_debug() const;

    void init_position_from_sysid();
    bool set_self_position_from_dir(direc d);

    void init_delayed_uart();
    void delayed_uart_tick();

    connection father;
    connection son[4];
    uint8_t backup_son[4];
    direc backup_dir[4];
    uint8_t son_max;
    uint8_t son_count;
    net_zero_position self_position;

    uint8_t  _delayed_uart_id;
    uint32_t _delayed_uart_enable_ms;
    bool     _delayed_uart_pending;
};

extern NetZeroRouter net_zero_router;

extern uint16_t SubDroneCache[10][4];

uint8_t get_mavlink_chan_by_uart(uint8_t uart_id);
uint8_t get_uart_id_by_mavlink_chan(uint8_t mavlink_chan);
uint8_t net_zero_dir_to_protocol(direc d);
direc net_zero_dir_from_protocol(uint8_t value);
net_zero_position net_zero_position_from_dir(direc d);

#define NETZERO_DELAYED_UART_ID        4
#define NETZERO_DELAYED_UART_ENABLE_MS 13000

void net_zero_delayed_uart_init();

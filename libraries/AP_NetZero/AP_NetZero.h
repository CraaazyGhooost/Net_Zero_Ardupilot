#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <stdint.h>

enum direc {
    front, left, back, right,
    no_dir = 0xFF
};

class connection {
public:
    bool valid;
    direc dir;
    uint8_t target_id;
    uint8_t serial_id;
    uint8_t mavlink_chan;
    connection() : valid(false), dir(front), target_id(0xFF), serial_id(0xFF), mavlink_chan(0xFF) {}
    connection(bool v, direc d, uint8_t t_id, uint8_t s_id, uint8_t m_ch) :
        valid(v),
        dir(d),
        target_id(t_id),
        serial_id(s_id),
        mavlink_chan(m_ch) {}
};

class NetZeroRouter {
public:
    NetZeroRouter();

    bool set_father_uart(direc d, uint8_t s_id, uint8_t t_id);
    bool add_son_uart(direc d, uint8_t s_id, uint8_t t_id);

    connection get_father();
    connection get_son(uint8_t son);
    uint8_t get_son_count();

    void init_delayed_uart();
    void delayed_uart_tick();

    connection father;
    connection son[4];
    uint8_t backup_son[4];
    direc backup_dir[4];
    uint8_t son_max;
    uint8_t son_count;

    uint8_t  _delayed_uart_id;
    uint32_t _delayed_uart_enable_ms;
    bool     _delayed_uart_pending;
};

extern NetZeroRouter net_zero_router;

extern uint16_t SubDroneCache[10][4];

uint8_t get_mavlink_chan_by_uart(uint8_t uart_id);
uint8_t get_uart_id_by_mavlink_chan(uint8_t mavlink_chan);

#define NETZERO_DELAYED_UART_ID        4
#define NETZERO_DELAYED_UART_ENABLE_MS 13000

void net_zero_delayed_uart_init();

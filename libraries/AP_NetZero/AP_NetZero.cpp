#include "AP_NetZero.h"

NetZeroRouter::NetZeroRouter()
{
    father = connection(false, front, 0xFF, 0xFF, 0xFF);
    for (int i = 0; i < 4; i++) {
        son[i] = connection(false, front, 0xFF, 0xFF, 0xFF);
    }
    backup_son[0] = 4;
    backup_son[1] = 0xFF;
    backup_son[2] = 5;
    backup_son[3] = 0xFF;
    backup_dir[0] = front;
    backup_dir[1] = left;
    backup_dir[2] = back;
    backup_dir[3] = right;
    self_position = net_zero_position();
    son_count = 0;
    son_max = 2;

    _delayed_uart_id = 0xFF;
    _delayed_uart_enable_ms = 0;
    _delayed_uart_pending = false;
}

void NetZeroRouter::init_delayed_uart()
{
    init_position_from_sysid();

    if (gcs().sysid_this_mav() != 1) {
        return;
    }
    if (gcs().sysid_this_mav() == 1 || gcs().sysid_this_mav() == 4) {
        return;
    }

    _delayed_uart_id = NETZERO_DELAYED_UART_ID;
    const AP_HAL::HAL& hal = AP_HAL::get_HAL();
    AP_HAL::UARTDriver *uart = hal.serial(_delayed_uart_id);

    if (uart != nullptr) {
        uart->end();
        hal.console->printf("NetZero: UART%u disabled (buffers cleared), "
                            "will re-enable in %ums\n",
                            _delayed_uart_id, NETZERO_DELAYED_UART_ENABLE_MS);

        _delayed_uart_enable_ms = AP_HAL::millis() + NETZERO_DELAYED_UART_ENABLE_MS;
        _delayed_uart_pending = true;

        hal.scheduler->register_timer_process(
            FUNCTOR_BIND(&net_zero_router, &NetZeroRouter::delayed_uart_tick, void));
    }
}

void NetZeroRouter::delayed_uart_tick()
{
    if (!_delayed_uart_pending) {
        return;
    }
    if (AP_HAL::millis() < _delayed_uart_enable_ms) {
        return;
    }

    const AP_HAL::HAL& hal = AP_HAL::get_HAL();
    AP_HAL::UARTDriver *uart = hal.serial(_delayed_uart_id);

    if (uart != nullptr) {
        const AP_SerialManager::UARTState *uart_state =
            AP::serialmanager().get_state_by_id(_delayed_uart_id);
        uint32_t baud = uart_state ? uart_state->baudrate() : 230400;

        uart->begin(baud,
                    AP_SERIALMANAGER_MAVLINK_BUFSIZE_RX,
                    AP_SERIALMANAGER_MAVLINK_BUFSIZE_TX);
        hal.console->printf("NetZero: UART%u re-enabled at %lu baud\n",
                            _delayed_uart_id, (unsigned long)baud);
    }

    _delayed_uart_pending = false;
}

bool NetZeroRouter::set_father_uart(direc d, uint8_t s_id, uint8_t t_id)
{
    if (s_id == 0xFF || t_id == 0xFF) {
        return false;
    }
    mavlink_channel_t mavlink_chan = (mavlink_channel_t)get_mavlink_chan_by_uart(s_id);
    if (mavlink_chan == 0xFF) {
        return false;
    }
    father = connection(true, d, t_id, s_id, mavlink_chan, net_zero_position(0, 0, true));
    return true;
}

bool NetZeroRouter::add_son_uart(direc d, uint8_t s_id, uint8_t t_id)
{
    if (d == direc::no_dir) {
        return false;
    }
    if (s_id == 0xFF || t_id == 0xFF) {
        return false;
    }
    if (son_count >= son_max) {
        return false;
    }
    for (uint8_t i = 0; i < son_count; i++) {
        if (son[i].valid && son[i].target_id == t_id) {
            return false;
        }
    }
    mavlink_channel_t mavlink_chan = (mavlink_channel_t)get_mavlink_chan_by_uart(s_id);
    if (mavlink_chan == 0xFF) {
        return false;
    }
    son[son_count] = connection(true, d, t_id, s_id, mavlink_chan, net_zero_position_from_dir(d));
    son_count++;
    GCS_MAVLINK::set_channel_private(mavlink_chan);
    for (int i = 0; i < 4; i++) {
        if (backup_son[i] == s_id) {
            backup_son[i] = 0xFF;
            break;
        }
    }
    return true;
}

connection NetZeroRouter::get_father()
{
    return father;
}

connection NetZeroRouter::get_son(uint8_t s)
{
    if (s < son_count) {
        return son[s];
    }
    return connection(false, front, 0xFF, 0xFF, 0xFF);
}

uint8_t NetZeroRouter::get_son_count()
{
    return son_count;
}

net_zero_position NetZeroRouter::get_self_position() const
{
    return self_position;
}

void NetZeroRouter::send_position_debug() const
{
    if (self_position.valid) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "Net zero position: sysid=%u pos=(%d,%d)",
                      gcs().sysid_this_mav(),
                      (int)self_position.x,
                      (int)self_position.y);
        return;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                  "Net zero position: sysid=%u pos=unknown",
                  gcs().sysid_this_mav());
}

void NetZeroRouter::init_position_from_sysid()
{
    if (gcs().sysid_this_mav() == 1) {
        self_position = net_zero_position(0, 0, true);
        return;
    }
    self_position = net_zero_position();
}

bool NetZeroRouter::set_self_position_from_dir(direc d)
{
    const net_zero_position pos = net_zero_position_from_dir(d);
    if (!pos.valid) {
        return false;
    }
    self_position = pos;
    return true;
}

NetZeroRouter net_zero_router;

uint16_t SubDroneCache[10][4];

uint8_t get_mavlink_chan_by_uart(uint8_t uart_id)
{
    const AP_HAL::HAL& hal = AP_HAL::get_HAL();
    AP_HAL::UARTDriver *target_uart = hal.serial(uart_id);
    if (target_uart == nullptr) {
        return 0xFF;
    }

    GCS_MAVLINK *target_link = nullptr;
    for (uint8_t j = 0; j < gcs().num_gcs(); j++) {
        GCS_MAVLINK *link = gcs().chan(j);
        if (link != nullptr && link->get_uart() == target_uart) {
            target_link = link;
            break;
        }
    }
    if (target_link != nullptr) {
        return target_link->get_chan();
    }
    return 0xFF;
}

uint8_t net_zero_dir_to_protocol(direc d)
{
    switch (d) {
    case front:
        return 0;
    case left:
        return 1;
    case back:
        return 2;
    case right:
        return 3;
    case no_dir:
    default:
        return 0xFF;
    }
}

direc net_zero_dir_from_protocol(uint8_t value)
{
    switch (value) {
    case 0:
        return front;
    case 1:
        return left;
    case 2:
        return back;
    case 3:
        return right;
    default:
        return no_dir;
    }
}

net_zero_position net_zero_position_from_dir(direc d)
{
    switch (d) {
    case front:
        return net_zero_position(0, 1, true);
    case left:
        return net_zero_position(-1, 0, true);
    case back:
        return net_zero_position(0, -1, true);
    case right:
        return net_zero_position(1, 0, true);
    case no_dir:
    default:
        return net_zero_position();
    }
}

uint8_t get_uart_id_by_mavlink_chan(uint8_t mavlink_chan)
{
    if (mavlink_chan >= gcs().num_gcs()) {
        return 0xFF;
    }
    GCS_MAVLINK *link = gcs().chan(mavlink_chan);
    if (link == nullptr) {
        return 0xFF;
    }
    AP_HAL::UARTDriver *target_uart = link->get_uart();
    if (target_uart == nullptr) {
        return 0xFF;
    }
    const AP_HAL::HAL& hal = AP_HAL::get_HAL();
    for (uint8_t i = 0; i < HAL_NUM_SERIAL_PORTS; i++) {
        if (hal.serial(i) == target_uart) {
            return i;
        }
    }
    return 0xFF;
}

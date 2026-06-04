#include "net_zero_protocol.h"
#include "AP_HAL/AP_HAL.h"

NetZeroRouter::NetZeroRouter() {
    // default constructor

    father = connection(false, front, 0xFF, 0xFF, 0xFF); // default father connection with invalid UART
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
    son_count = 0;
    son_max = 2;

    // delayed UART enable state
    _delayed_uart_id = 0xFF;
    _delayed_uart_enable_ms = 0;
    _delayed_uart_pending = false;
}

void NetZeroRouter::init_delayed_uart()
{
    _delayed_uart_id = NETZERO_DELAYED_UART_ID;
    const AP_HAL::HAL& hal = AP_HAL::get_HAL();
    AP_HAL::UARTDriver *uart = hal.serial(_delayed_uart_id);

    if (uart != nullptr) {
        /*
         * use end() to fully deinit the UART:
         *   - stops DMA, serial hardware, and threads
         *   - clears TX/RX buffers (discards all dead-period data,
         *     preventing overflow since TX buffer is only 256 bytes)
         *   - sets _tx_initialised/_rx_initialised to false
         *
         * MAVLink comm_send_buffer() safely ignores write failures
         * on real hardware (see GCS_MAVLink.cpp line 149-156):
         *   const size_t written = ...write(buf, len);
         *   (void)written;  // short write silently dropped on ChibiOS
         */
        uart->end();
        hal.console->printf("NetZero: UART%u disabled (buffers cleared), "
                            "will re-enable in %ums\n",
                            _delayed_uart_id, NETZERO_DELAYED_UART_ENABLE_MS);
    }

    _delayed_uart_enable_ms = AP_HAL::millis() + NETZERO_DELAYED_UART_ENABLE_MS;
    _delayed_uart_pending = true;

    hal.scheduler->register_timer_process(
        FUNCTOR_BIND(&net_zero_router, &NetZeroRouter::delayed_uart_tick, void));
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
        // get baudrate from serial manager state (set by SERIALn_BAUD parameter)
        const AP_SerialManager::UARTState *uart_state =
            AP::serialmanager().get_state_by_id(_delayed_uart_id);
        uint32_t baud = uart_state ? uart_state->baudrate() : 57600;

        // begin() re-allocates buffers, restarts DMA & serial hardware,
        // and restores pins to their alternate function. Stale data is
        // already discarded by end(), so we start completely fresh.
        uart->begin(baud,
                    AP_SERIALMANAGER_MAVLINK_BUFSIZE_RX,
                    AP_SERIALMANAGER_MAVLINK_BUFSIZE_TX);
        hal.console->printf("NetZero: UART%u re-enabled at %lu baud\n",
                            _delayed_uart_id, (unsigned long)baud);
    }

    _delayed_uart_pending = false;  // one-shot, only execute once
}

bool NetZeroRouter::set_father_uart(direc d, uint8_t s_id, uint8_t t_id) {
    if (s_id == 0xFF || t_id == 0xFF) {
        return false; // Invalid UART
    }
    mavlink_channel_t mavlink_chan = (mavlink_channel_t)get_mavlink_chan_by_uart(s_id);
    if(mavlink_chan == 0xFF) {
        return false; // UART exists but is not a MAVLink channel
    }
     // set the father connection with the provided details and the corresponding MAVLink channel
    father = connection(true, d, t_id, s_id, mavlink_chan);
    return true;
}

bool NetZeroRouter::add_son_uart(direc d, uint8_t s_id, uint8_t t_id) {
    if(d == direc::no_dir){
        return false; // Invalid direction
    }
    if (s_id == 0xFF || t_id == 0xFF) {
        return false; // Invalid UART
    }
    if( son_count >= son_max) {
        // Maximum number of son UARTs reached, cannot add more
        return false;
    }
    mavlink_channel_t mavlink_chan = (mavlink_channel_t)get_mavlink_chan_by_uart(s_id);
    if(mavlink_chan == 0xFF){
        return false; // UART exists but is not a MAVLink channel
    }
    son[son_count] = connection(true, d, t_id, s_id, mavlink_chan);
    son_count++;
    for(int i = 0; i < 4; i++){
        if(backup_son[i] == s_id){
            backup_son[i] = 0xFF; // clear the backup
            break;
        }
    }
    return true;
}

connection NetZeroRouter::get_father() {
    return father;
}

connection NetZeroRouter::get_son(uint8_t s) {
    if (s < son_count) {
        return son[s];
    } else {
        return connection(false, front, 0xFF, 0xFF, 0xFF); // Invalid son index
    }
}

uint8_t NetZeroRouter::get_son_count() { 
    return son_count;
}

NetZeroRouter net_zero_router; // global instance

uint16_t SubDroneCache[10][4];

uint8_t get_mavlink_chan_by_uart(uint8_t uart_id){
    const AP_HAL::HAL& hal = AP_HAL::get_HAL();
    AP_HAL::UARTDriver *target_uart = hal.serial(uart_id);
    if (target_uart == nullptr) { 
        // hal.console->printf("UART %d is not available\n", uart_id);
        return 0xFF; // Invalid UART
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
    } else {
        // hal.console->printf("Serial %d is not a MAVLink channel\n", uart_id);
        return 0xFF; // UART exists but is not a MAVLink channel
    }
}

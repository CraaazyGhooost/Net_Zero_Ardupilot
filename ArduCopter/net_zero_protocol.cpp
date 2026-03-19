#include "net_zero_protocol.h"

NetZeroRouter::NetZeroRouter() {
    // default constructor

    father = connection(false, front, 0xFF, 0xFF); // default father connection with invalid UART
    for (int i = 0; i < 4; i++) {
        son[i] = connection(false, front, 0xFF, 0xFF);
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
}

bool NetZeroRouter::set_father_uart(direc d, uint8_t s_id, uint8_t t_id) {
    if (s_id == 0xFF || t_id == 0xFF) {
        return false; // Invalid UART
    }
    father = connection(true, d, t_id, s_id);
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
    son[son_count] = connection(true, d, t_id, s_id);
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
        return connection(false, front, 0xFF, 0xFF); // Invalid son index
    }
}

uint8_t NetZeroRouter::get_son_count() { 
    return son_count;
}

NetZeroRouter net_zero_router; // global instance
#include "RSSInterface.hpp"
#include "CoreRDCCANInterface.hpp"

void RSSInterface::receiveRSSBootCANmsg(const CAN_message_t &msg, unsigned long long millis)
{
    // need to set the operational mode once boot msg is received
    enqueueSetOperationalCANMsg();
}

void RSSInterface::receiveRSSStatusCANMsg(const CAN_message_t &msg, unsigned long long millis)
{
    // cast to RSS state msg
    RSS_STATUS_t status_msg;
    Unpack_RSS_STATUS_ht_can(&status_msg, &msg.buf[0], msg.len);

    // populate interface data with received data
    _data.is_go_button_pressed = status_msg.button_k3_pressed;
    _data.is_go_switch_on = status_msg.switch_k2_pressed;
    _data.is_estop_pressed = status_msg.emergency_stop_not_pressed_1 && status_msg.emergency_stop_not_pressed_2;
    _data.is_correct_mode_selected = status_msg.correct_mode_selected;
    _data.is_pre_alarm_warning_active = status_msg.pre_alarm_warning;

    _data.radio_link_quality = status_msg.radio_link_quality;

    _data.last_recv_millis = millis;
}

void RSSInterface::enqueueSetOperationalCANMsg() {}

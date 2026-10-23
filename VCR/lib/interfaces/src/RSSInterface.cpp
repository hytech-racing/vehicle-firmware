#include "RSSInterface.hpp"
#include "VCRCANInterfaceImpl.hpp"

void RSSInterface::receiveStatusMessage(const CAN_message_t &msg)
{
    RSS_STATUS_t status_msg;
    Unpack_RSS_STATUS_ht_can(&status_msg, &msg.buf[0], msg.len);

    // populate interface data with received data
    _rss_data.is_go_button_pressed = status_msg.button_k3_pressed;
    _rss_data.is_go_switch_on = status_msg.switch_k2_pressed;
    _rss_data.is_estop_pressed = status_msg.emergency_stop_not_pressed_1 && status_msg.emergency_stop_not_pressed_2;
    _rss_data.is_correct_mode_selected = status_msg.correct_mode_selected;
    _rss_data.is_pre_alarm_warning_active = status_msg.pre_alarm_warning;

    _rss_data.radio_link_quality = status_msg.radio_link_quality;
}

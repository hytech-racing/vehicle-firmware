#ifndef RSS_INTERFACE_H
#define RSS_INTERFACE_H

#include "CANInterface.h"
#include "FlexCAN_T4.h"
#include "ht_can.h"
#include <etl/singleton.h>

constexpr uint8_t OPERATIONAL_MODE_STATE = 0x01;

struct RSSData_s
{
    uint64_t last_recv_millis;
    uint8_t radio_link_quality;
    bool is_go_button_pressed;
    bool is_go_switch_on;
    bool is_estop_pressed;
    bool is_correct_mode_selected;
    bool is_pre_alarm_warning_active;
};

class RSSInterface
{
    public:
    RSSInterface() = delete;
    RSSInterface(uint8_t node_id)
        : _node_id(node_id)
    {
    }

    void receiveRSSBootCANmsg(const CAN_message_t &msg, unsigned long long millis);
    void receiveRSSStatusCANMsg(const CAN_message_t &msg, unsigned long long millis);
    void enqueueSetOperationalCANMsg();

    bool isGoButtonPressed() const { return _data.is_go_button_pressed; }
    bool isGoSwitchOn() const { return _data.is_go_switch_on; }
    bool isEStopPressed() const { return _data.is_estop_pressed; }
    uint8_t getRadioLinkQuality() const { return _data.radio_link_quality; }
    bool isCorrectModeSelected() const { return _data.is_correct_mode_selected; }
    bool isPreAlarmWarningActive() const { return _data.is_pre_alarm_warning_active; }
    uint64_t getLastRecvMillis() const { return _data.last_recv_millis; }
    uint8_t getNodeID() const { return _node_id; }

    private:
    uint8_t _node_id;
    RSSData_s _data;
};

using RSSInterfaceInstance = etl::singleton<RSSInterface>;
#endif // RSS_INTERFACE_H

#ifndef RSS_INTERFACE_HPP
#define RSS_INTERFACE_HPP

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include "SharedFirmwareTypes.h"
#include "ht_can.h"
#include "shared_types.hpp"
#include <FlexCAN_T4.h>

/* Local Interface Includes */
#include "SystemTimeInterface.h"

struct RSSCANInterfaceData_s
{
    uint8_t is_go_button_pressed;
    uint8_t is_go_switch_on;
    uint8_t is_estop_pressed;
    uint8_t is_correct_mode_selected;
    uint8_t is_pre_alarm_warning_active;
    uint8_t radio_link_quality;
};

class RSSInterface
{

    public:
    RSSInterface()
        : _rss_data{} {};

    void receiveStatusMessage(const CAN_message_t &msg);

    bool isGoButtonPressed() const { return _rss_data.is_go_button_pressed; };

    bool isGoSwitchOn() const { return _rss_data.is_go_switch_on; };

    bool isEstopPressed() const { return _rss_data.is_estop_pressed; };

    bool isCorrectModeSelected() const { return _rss_data.is_correct_mode_selected; };

    bool isPreAlarmWarningActive() const { return _rss_data.is_pre_alarm_warning_active; };

    uint8_t getRadioLinkQuality() const { return _rss_data.radio_link_quality; };

    private:
    RSSCANInterfaceData_s _rss_data;
};

using RSSInterfaceInstance = etl::singleton<RSSInterface>;

#endif

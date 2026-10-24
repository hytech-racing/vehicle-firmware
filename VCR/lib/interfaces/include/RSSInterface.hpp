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
    RSSInterface();

    void receive_status_message(const CAN_message_t& msg);

    bool is_go_button_pressed() { return _rss_data.is_go_button_pressed; };

    bool is_go_switch_on() { return _rss_data.is_go_switch_on; };

    bool is_estop_pressed() { return _rss_data.is_estop_pressed; };

    bool is_correct_mode_selected() { return _rss_data.is_correct_mode_selected; };

    bool is_pre_alarm_warning_active() { return _rss_data.is_pre_alarm_warning_active; };

    uint8_t radio_link_quality() { return _rss_data.radio_link_quality; };

  private:
    mutable RSSCANInterfaceData_s _rss_data;
};


using RSSInterfaceInstance = etl::singleton<RSSInterface>;

#endif
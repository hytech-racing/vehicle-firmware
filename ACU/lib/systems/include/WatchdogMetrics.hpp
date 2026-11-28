#ifndef WATCHDOG_METRICS_H
#define WATCHDOG_METRICS_H
#include "SharedFirmwareTypes.h"
#include "etl/singleton.h"

class WatchdogMetrics
{
  public:
    struct WatchdogMetricsData_s
    {
        Volt max_measured_glv;
        Volt max_measured_pack_out_voltage;
        Volt max_measured_ts_out_voltage;

        Volt min_measured_glv;
        Volt min_measured_pack_out_voltage;
        Volt min_measured_ts_out_voltage;

        Volt min_shdn_out_voltage;

        unsigned long last_valid_shdn_out_ms;
    };


    void update_metrics(Volt measured_glv, Volt mesaured_pack_out_voltage, Volt measured_ts_out_voltage, Volt shdn_out_voltage, unsigned long millis);

    void reset_metrics(Volt measured_glv, Volt measured_pack_out_voltage, Volt measured_ts_out_voltage, Volt shdn_out_voltage);

    bool is_shdn_out_voltage_invalid(unsigned long millis);

    WatchdogMetricsData_s get_watchdog_metrics();

  private:
    static constexpr float VALID_SHDN_OUT_MIN_VOLTAGE_THRESHOLD = 12.0F;
    static constexpr uint32_t MIN_ALLOWED_INVALID_SHDN_OUT_MS = 10;
    WatchdogMetricsData_s _watchdog_metrics_data;
};

using WatchdogMetricsInstance = etl::singleton<WatchdogMetrics>;

#endif

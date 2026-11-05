#ifndef BUCK_CONVERTER_INTERFACE_HPP
#define BUCK_CONVERTER_INTERFACE_HPP

#include <cstdint>
#include <array>
#include <Arduino.h>
#include "SharedFirmwareTypes.h"
#include <etl/singleton.h>

/**
 * @file BuckConverterInterface.hpp
 * @brief Driver for the PDB's DC/DC converters. Each one has an optional enable pin (EN / RUN)
 *        driven by the MCU and an optional open-drain power-good pin (PG / RESET) read by the MCU.
 *
 *  - TPS62085
 *      (5V -> 3V3 buck)
 *      Control: EN turns it on and off.
 *      Status:  PG is high when the output is above ~95% of its target and is low when it drops below ~90%.
 *      NOTES: Scenarios when PG goes low:
 *                1) It's off
 *                2) Thermal shutdown (~150 °C) automatically back on when 20 °C cooler
 *                3) Short-circuit hiccup (after 32 current-limit hits it stops, then retries after ~66 µs).
 *             Shutdown: Output is discharged through an internal 260 Ohm resistor (sets RESTART_OFF_MS).
 *
 *  - LM61495T-Q1
 *      (24V -> 12V / 18V)
 *      Control: EN turns it on and off.
 *      Status:  RESET works like PG.
 *      NOTES: Scenarios when RESET goes low
 *                1) Output is under 94% or over 112% (the overvoltage case also stops switching
 *                   until the output falls back below the threshold)
 *                2) It's off
 *                3) In soft start
 *                4) Thermal shutdown (168 °C) automatically back on at 159 °C
 *                5) Short-circuit hiccup (waits 40 ms, then soft-starts again). Only triggers when the output
 *                   is below ~40% of target for 128 cycles after soft start; a milder overload just current-limits.
 *             Filtering: Chip filters glitches itself (26 µs). After a fault clears, it holds RESET low for
 *                        another 2.1 ms (3.4 ms max) before reporting good. Not applied on the first startup.
 *             Shutdown: No output discharge; the output drains through the load.
 *
 *  - LTC3115-2
 *      (24V -> 24V buck-boost)
 *      Control: RUN, which is its EN.
 *      Status:  None :)
 *      Mode:    PWM/SYNC is tied to GND, so it always runs in Burst Mode
 *      NOTES:  Can only assume the rail is up once the soft-start time (9 ms nominal) has passed. It can't detect faults.
 *                1) Thermal shutdown (~165 °C) automatically back on once ~10 °C cooler, with a fresh soft start
 *                2) No hiccup: overload keeps running at the ~3 A average current limit (halved if Vout < 1.85 V).
 *                   A sustained short can heat it into thermal shutdown and make it cycle.
 *             Shutdown: Output is disconnected from the input; it drains through the load.
 *
 *  - MAXM17536
 *      (12V -> 5V)
 *      Control: EN/UVLO. Has an internal 3.32 MOhm pullup to VIN, so the rail turns on if EN floats.
 *      Status:  RESET works like PG. No overvoltage detection.
 *      NOTES: Scenarios when RESET goes low
 *                1) Output is under 92.5% (goes high again 1024 switching cycles after it rises above 95.5%)
 *                2) It's off
 *                3) In soft start
 *                4) Thermal shutdown (165 °C) automatically back on once 10 °C cooler
 *                5) Hiccup: triggers on one hit of the 8.8 A runaway current limit, or the output below 64.5%
 *                   of target (FB < 0.58 V) after soft start. It stays off for 32768 cycles, then soft-starts again.
 *            Soft start: set by the board's C_SS capacitor: t_SS[ms] = C_SS[nF] / 5.55.
 *            Shutdown: No output discharge; the output drains through the load.
 *
 *  All parts restart on their own after every fault above; none latch off. All also turn back on
 *  automatically when their input recovers above UVLO.
*/

namespace tps62085_params
{
    constexpr uint32_t PG_DEBOUNCE_MS = 5;          // PG must hold a new level this long before we accept it
    constexpr uint32_t STARTUP_TIMEOUT_MS = 5;      // 0.8 ms typical soft start
    constexpr uint32_t RESTART_OFF_MS = 35;         // tau = 260 * 22.1uF = 5.75ms -> 5 tau (28.6) is 0.022 V
}

namespace lm61495_params
{
    constexpr uint32_t PG_DEBOUNCE_MS = 2;          // On-chip 26 µs deglitch already; this only filters board noise
    constexpr uint32_t STARTUP_TIMEOUT_MS = 5;      // t_EN (1.2 ms max) + t_SS (2.7 ms max), padded incase
    constexpr uint32_t RESTART_OFF_MS = 100;        /// No output discharge: Vout decays through the load
    /// TODO: size for the rail
}

namespace ltc3115_params
{
    constexpr uint32_t STARTUP_TIMEOUT_MS = 15;     // 9 ms nominal soft start (no min/max given), padded
    constexpr uint32_t RESTART_OFF_MS = 100;        // Output disconnect in shutdown: Vout decays through the load
    /// TODO: size for the rail
}

namespace maxm17536_params
{
    constexpr uint32_t PG_DEBOUNCE_MS = 1;          // On-chip 2.3ms deglitch already; this only filters board noise
    constexpr uint32_t RESTART_OFF_MS = 100;        // No output discharge: Vout decays through the load
    /// TODO: size for the rail

    // Board values
    constexpr float SS_CAP_NF = 22.0f;              // C_SS capacitor
    constexpr float SWITCHING_FREQ_KHZ = 450.0f;    // RT open = 450 kHz default

    /**
     * @brief Startup timeout from the board's soft-start cap and switching frequency, with 2x margin.
    */
    constexpr uint32_t computeStartupTimeoutMs()
    {
        return static_cast<uint32_t>(2.0f * (SS_CAP_NF / 5.55f + 1024.0f / SWITCHING_FREQ_KHZ)) + 1;
    }
}

namespace buck_converter_interface_default_params
{
    constexpr uint8_t NUM_BUCKS = 6;
}

/**
 * @enum BuckConverterIDs_e
 * @brief Index of each converter
*/
enum BuckConverterIDs_e : uint8_t
{
    BUCK_LIDAR_24V = 0,
    BUCK_ORIN_18V  = 1,
    BUCK_DTI_12V   = 2,
    BUCK_MAIN_12V  = 3,
    BUCK_MAIN_5V   = 4,
    BUCK_MAIN_3V3  = 5,
    NUM_BUCK_IDS
};
static_assert(NUM_BUCK_IDS == buck_converter_interface_default_params::NUM_BUCKS, "BuckConverterIDs_e must list every buck");

/**
 * @enum BuckConverterPart_e
 * @brief Converter part number. Completely informational
*/
enum class BuckConverterPart_e : uint8_t
{
    TPS62085,
    LM61495,
    LTC3115,
    MAXM17536,
    NUM_PARTS
};

/**
 * @enum BuckConverterState_e
 * @brief States a converter can be in, derived from the EN and PG pins
 * @param DISABLED EN low
 * @param STARTING EN high, waiting for PG (or for the soft-start time if there is no PG pin)
 * @param ACTIVE EN high, PG high (or soft-start time elapsed if there is no PG pin)
 * @param FAULT EN high, PG low; the part may recover by itself (thermal / hiccup). Never entered without a PG pin.
 * @param RESTARTING EN held low so the output discharges, then re-enabled automatically
 * @note Rails without an EN pin are always on: they start in STARTING at init() and never reach DISABLED / RESTARTING.
*/
enum class BuckConverterState_e : uint8_t
{
    DISABLED,
    STARTING,
    ACTIVE,
    FAULT,
    RESTARTING,
    NUM_STATES
};

/**
 * @enum BuckConverterFaultType_e
 * @brief Likely cause when a converter is in the FAULT state.
 * @note PG alone cannot distinguish the individual causes within a category.
 * @param NONE No fault
 * @param STARTUP_TIMEOUT PG never rose after enable
 * @param POWER_LOST PG was high, then dropped: overload (hiccup), thermal shutdown, VIN brownout below UVLO,
 *                   or (LM61495 only) output overvoltage
*/
enum class BuckConverterFaultType_e : uint8_t
{
    NONE,
    STARTUP_TIMEOUT,
    POWER_LOST,
    NUM_FAULTS
};

/**
 * @struct BuckConverterTiming_s
 * @brief Timing parameters for one converter. Per-part defaults are in the *_params namespaces.
 * @param startup_timeout_duration_ms With PG: max time from EN high to PG high before STARTUP_TIMEOUT is raised.
 *                                    Without PG: time after EN high at which the rail is assumed ACTIVE.
 * @param pg_debounce_duration_ms How long PG must hold a new level before the debounced value follows it. Unused without PG.
 * @param restart_wait_duration_ms Default EN-low time for restart(); size it so the output discharges
 *                                 below the downstream reset threshold.
*/
struct BuckConverterTiming_s
{
    uint32_t startup_timeout_duration_ms;
    uint32_t pg_debounce_duration_ms;
    uint32_t restart_wait_duration_ms;
};

/**
 * @brief Datasheet timing defaults per part. A rail can use these directly or override fields
 *        (e.g. restart_wait_duration_ms sized for its output caps).
*/
namespace buck_converter_default_timings
{
    constexpr BuckConverterTiming_s TPS62085 = {
        .startup_timeout_duration_ms = tps62085_params::STARTUP_TIMEOUT_MS,
        .pg_debounce_duration_ms = tps62085_params::PG_DEBOUNCE_MS,
        .restart_wait_duration_ms = tps62085_params::RESTART_OFF_MS
    };

    constexpr BuckConverterTiming_s LM61495 = {
        .startup_timeout_duration_ms = lm61495_params::STARTUP_TIMEOUT_MS,
        .pg_debounce_duration_ms = lm61495_params::PG_DEBOUNCE_MS,
        .restart_wait_duration_ms = lm61495_params::RESTART_OFF_MS
    };

    constexpr BuckConverterTiming_s LTC3115 = {
        .startup_timeout_duration_ms = ltc3115_params::STARTUP_TIMEOUT_MS,
        .pg_debounce_duration_ms = 0,
        .restart_wait_duration_ms = ltc3115_params::RESTART_OFF_MS
    };
}

namespace buck_converter_interface_default_params
{
    constexpr uint32_t NOT_CONNECTED = NC;      // EN or PG not wired to the MCU (Arduino "no pin")
}

/**
 * @struct BuckConverterDefinition_s
 * @brief Setup (pin/wiring and timing) of one converter, passed to the constructor.
 * @param id Which slot in the array this buck converter fills
 * @param part Part number
 * @param en EN / RUN pin (MCU output, Arduino pin from Pins.h). NOT_CONNECTED if the rail is always on
 * @param pg PG / RESET pin (MCU input, Arduino pin from Pins.h). NOT_CONNECTED if the part has none
 * @param timing_params Timing parameters, usually from buck_converter_default_timings
*/
struct BuckConverterDefinition_s
{
    BuckConverterIDs_e id;
    BuckConverterPart_e part;
    uint32_t en;
    uint32_t pg;
    BuckConverterTiming_s timing_params;
};

/**
 * @struct BuckConverterStatus_s
 * @brief State, health and counters for one converter
 * @param part Part number of this converter
 * @param current_state Current state of the converter
 * @param current_fault Cause of the fault if current_state is FAULT, otherwise NONE
 * @param has_en_pin False if the rail is always on (EN not connected to the MCU)
 * @param has_pg_pin False if the part has no PG; ACTIVE is then assumed, not measured
 * @param is_enabled True if EN is driven high, or always for rails without an EN pin
 * @param is_pg_high Debounced PG level. Always false without a PG pin
 * @param fault_count Number of faults since boot
 * @param recovery_count Number of times the part went from FAULT back to ACTIVE on its own
*/
struct BuckConverterStatus_s
{
    BuckConverterPart_e part;
    BuckConverterState_e current_state;
    BuckConverterFaultType_e current_fault;
    bool has_en_pin;
    bool has_pg_pin;
    bool is_enabled;
    bool is_pg_high;
    uint32_t fault_count;
    uint32_t recovery_count;
};

/**
 * @brief Represents one converter
 * @param en_pin EN (or RUN) pin, NOT_CONNECTED if the rail is always on
 * @param pg_pin PG (or RESET) pin, NOT_CONNECTED if the part has none
 * @param timing_params Timing parameters, see BuckConverterTiming_s
 * @param is_pg_raw_high Last sampled PG pin level (raw)
 * @param pg_raw_changed_at_ms Set when is_pg_raw_high flips; debounce timer counts from here
 * @param enabled_at_ms Set when EN goes high (or at init() if always on); startup timeout and duration count from here
 * @param restart_began_at_ms Set in restart() when EN is pulled low; re-enable happens timing_params.restart_wait_duration_ms later
 * @param status State, health and counters, see BuckConverterStatus_s
*/
struct BuckConverter_s
{
    uint32_t en_pin;
    uint32_t pg_pin;
    BuckConverterTiming_s timing_params;
    bool is_pg_raw_high;
    uint32_t pg_raw_changed_at_ms;
    uint32_t enabled_at_ms;
    uint32_t restart_began_at_ms;
    BuckConverterStatus_s status;
};

/**
 * @class BuckConverterInterface
 * @brief Controls and monitors all PDB converters through their EN and PG pins.
 *
 * Call init() once after GPIO init, then updateAllBuckConverters() periodically (~1 ms) so PG is sampled,
 * debounced, and every state is kept current.
 */
class BuckConverterInterface
{
public:

    /**
     * @brief Registers all converters. Does not touch the GPIOs, so it is safe to construct
     *        before HAL / GPIO init.
     * @param definitions One entry per converter, in any order; each is placed by its id.
     *        A duplicate or out-of-range id makes the config invalid and init() refuses to run.
    */
    explicit BuckConverterInterface(const std::array<BuckConverterDefinition_s, buck_converter_interface_default_params::NUM_BUCKS> &definitions);

    BuckConverterInterface(const BuckConverterInterface &) = delete;
    BuckConverterInterface &operator=(const BuckConverterInterface &) = delete;

    /**
     * @brief Puts every converter in its starting state.
     * @note Rails with an EN pin are driven low (DISABLED) until enable() is called.
     *       Always-on rails go to STARTING, so their startup timeout counts from now.
     * @return HAL_OK, or HAL_ERROR if the definitions passed to the constructor were invalid
    */
    HAL_StatusTypeDef init();

    /**
     * @brief Enables one converter. Drives EN high, starts the startup timer and sets state to STARTING.
     * @note No-op if already enabled or a restart() is in progress.
     * @return HAL_OK if enabled (or already on), HAL_ERROR if the index is invalid or the rail has no EN pin
    */
    HAL_StatusTypeDef enable(uint8_t index);

    /**
     * @brief Disables one converter. Drives EN low and sets state to DISABLED.
     * @return HAL_OK, or HAL_ERROR if the index is invalid or the rail has no EN pin
    */
    HAL_StatusTypeDef disable(uint8_t index);

    /**
     * @brief Disable, wait for the output to discharge, then enable again. updateBuckConverter() re-enables
     *        once the rail's timing_params.restart_wait_duration_ms has passed.
     * @return HAL_OK, or HAL_ERROR if the index is invalid or the rail has no EN pin
    */
    HAL_StatusTypeDef restart(uint8_t index);

    /**
     * @brief Samples PG, debounces it, and updates the state and counters of one converter.
    */
    void updateBuckConverter(uint8_t index);

    /**
     * @brief updateBuckConverter() for every converter. Call every ~1 ms.
    */
    void updateAllBuckConverters();

    /**
     * @return Status of the buck if the index is valid, nullptr otherwise
    */
    const BuckConverterStatus_s *getStatus(uint8_t index) const;

    /**
     * @return State of the buck if the index is valid, set to DISABLED otherwise
    */
    BuckConverterState_e getState(uint8_t index) const;

    /**
     * @note Return value is based on the state, thus PG is debounced here
     * @return True only when ACTIVE.
    */
    bool isPowerGood(uint8_t index) const;

    /**
     * @note Instant pin read, no debounce.
     * @return True if the index is valid and the pin is set; False without a PG pin
    */
    bool isPowerGoodPinHigh(uint8_t index) const;

    /**
     * @note Only check if the ENABLED bucks are is ACTIVE (PG is good)
    */
    bool allEnabledPowerGood() const;

    /**
     * @note Checks all buck regardless if they have PG pin or not
    */
    bool anyFault() const;

    /**
     * @return False if the constructor saw a duplicate or out-of-range id
    */
    bool isConfigValid() const { return _is_config_valid; }

private:

    BuckConverter_s _all_bucks[buck_converter_interface_default_params::NUM_BUCKS];
    bool _is_config_valid = true;

    /**
     * @brief Does what it says.
    */
    bool _isValidIndex(uint8_t index) const;

    /**
     * @brief Writes the EN pin. Does nothing for rails without an EN pin.
     * @param level is the desired state for EN, True = SET/HIGH, False = RESET/LOW
    */
    static void _setEnablePin(const BuckConverter_s &buck, bool level);

    /**
     * @brief Raw PG pin read.
     * @return True if the pin is set/HIGH, false for parts without a PG pin or if not set.
     * @note Instant read, no debounce; _updatePowerGood() does the debouncing.
    */
    static bool _isPowerGoodPinSet(const BuckConverter_s &buck);

    /**
     * @brief Turns a converter on: drives EN high, marks it enabled, and moves it to STARTING.
     * @note No checks. enable() validates and applies its guards before calling this; updateBuckConverter() calls it
     *       directly to end a restart, so enable()'s RESTARTING guard doesn't block it.
    */
    static void _turnOn(BuckConverter_s &buck, time_ms curr_millis);

    /**
     * @brief Moves a converter to STARTING and restarts its startup and debounce timers.
    */
    static void _beginStartup(BuckConverter_s &buck, time_ms curr_millis);

    /**
     * @brief Debounces PG and updates status.is_pg_high.
     * @note Debounces in both directions, high to low and low to high: PG must hold a new level
     *       for pg_debounce_duration_ms before status.is_pg_high follows it.
     * @warning Every real change is reported about 2–3 ms late due to the debounce period. The critical
     *          one is a fault: it may be reported after the hardware is already off.
    */
    static void _updatePowerGood(BuckConverter_s &buck, time_ms curr_millis);

    /**
     * @brief Records a fault, moves state to FAULT, and increments fault counter.
     *        EN stays high so the part can recover on its own.
    */
    static void _enterFault(BuckConverter_s &buck, BuckConverterFaultType_e fault);

};
using BuckConverterInterfaceInstance = etl::singleton<BuckConverterInterface>;

#endif // BUCK_CONVERTER_INTERFACE_HPP
#ifndef DASH_SCREENS_H
#define DASH_SCREENS_H

/**
 * Screen selection for the driver dashboard.
 *
 * Hardware-independent on purpose (no HAL, no Arduino, no display): it only maps the dial position to a
 * screen and debounces it, so it can be unit tested natively like the other "systems".
 */

/* Standard Library */
#include <stdint.h>

/* External Includes */
#include "SharedFirmwareTypes.h"

enum class DashScreen_e : uint8_t
{
    STANDARD,     // the original all-purpose layout
    ACCELERATION, // short run: speed only, nothing else competing for attention
    HANDLING,     // skidpad + autocross: speed, temps, SoC
    ENDURANCE,    // SoC in big numbers, SoE, speed, temps
    COOLING,      // pit / cooling modes: temperatures and cell voltage up front
};

namespace DashScreens
{
    constexpr uint8_t NUM_DIAL_POSITIONS = 6;

    /**
     * ============================  EDIT THIS TABLE  ============================
     * One entry per dial position (ControllerMode_e::MODE_0 .. MODE_5), in order.
     *
     * Why these defaults (from VCR/src/controls.cpp and VCR_InterfaceTasks.cpp):
     *   MODE_0  TorqueControllerSimple                         -> ENDURANCE (also the boot / between-detents default)
     *   MODE_1  LoadCellVectoringTorqueController              -> HANDLING
     *   MODE_2  TorqueControllerSimple, motor+inverter cooling -> ENDURANCE
     *   MODE_3  SimpleLaunchController                         -> ACCELERATION
     *   MODE_4  DrivebrainController                           -> STANDARD (no driverless screen yet)
     *   MODE_5  no torque controller, inverter cooling forced  -> COOLING
     * These are guesses from the controller names. Change them to whatever the team actually uses.
     */
    constexpr DashScreen_e SCREEN_FOR_MODE[NUM_DIAL_POSITIONS] = {
        DashScreen_e::ENDURANCE,    // MODE_0
        DashScreen_e::HANDLING,     // MODE_1
        DashScreen_e::ENDURANCE,    // MODE_2
        DashScreen_e::ACCELERATION, // MODE_3
        DashScreen_e::STANDARD,     // MODE_4
        DashScreen_e::COOLING,      // MODE_5
    };

    /// Frames a new dial position must be seen in a row before the screen changes. VCF reports MODE_0 whenever
    /// no dial contact is made, so a knob turning between detents would otherwise flash the MODE_0 screen.
    /// 6 frames = 200 ms at the 30 Hz screen refresh.
    constexpr uint8_t DIAL_CONFIRM_FRAMES = 6;

    inline DashScreen_e screen_for_mode(ControllerMode_e mode)
    {
        const auto idx = static_cast<uint8_t>(mode);
        return (idx < NUM_DIAL_POSITIONS) ? SCREEN_FOR_MODE[idx] : DashScreen_e::STANDARD;
    }

    /// Everything a screen needs, collected once per frame so the draw code never talks to the CAN interfaces.
    struct ScreenData_s
    {
        float speed_mph = 0.0f;
        float soc_percent = -1.0f; // negative = no data yet
        float soe_percent = -1.0f; // negative = no data yet
        float min_cell_voltage = 0.0f;
        float pack_voltage = 0.0f;
        int inverter_max_temp = 0;
        int motor_max_temp = 0;
        ControllerMode_e dial_mode = ControllerMode_e::MODE_0;
    };

    /// Debounced dial -> screen mapping. Call update() once per frame with the raw dial position.
    class ScreenSelector
    {
    public:
        explicit ScreenSelector(uint8_t confirm_frames = DIAL_CONFIRM_FRAMES) : _confirm_frames(confirm_frames) {}

        DashScreen_e update(ControllerMode_e raw_mode)
        {
            if (raw_mode == _candidate)
            {
                if (_seen < 0xFF)
                {
                    ++_seen;
                }
            }
            else
            {
                _candidate = raw_mode;
                _seen = 1;
            }

            if (_seen >= _confirm_frames)
            {
                _confirmed = _candidate;
            }
            return screen_for_mode(_confirmed);
        }

        /// The dial position the current screen was chosen for (what to print as the mode number).
        ControllerMode_e confirmed_mode() const { return _confirmed; }

    private:
        uint8_t _confirm_frames;
        uint8_t _seen = 0;
        ControllerMode_e _candidate = ControllerMode_e::MODE_0;
        ControllerMode_e _confirmed = ControllerMode_e::MODE_0;
    };
} // namespace DashScreens

#endif /* DASH_SCREENS_H */

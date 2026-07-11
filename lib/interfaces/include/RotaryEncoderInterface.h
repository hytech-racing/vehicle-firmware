#ifndef ROTARYENCODERINTERFACE_H
#define ROTARYENCODERINTERFACE_H

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include "SharedFirmwareTypes.h"

/* Local Interface Includes */
#include "ButtonInterface.h"

using pin = uint8_t;


namespace default_encoder_params
{
    constexpr float INIT_ENCODER_VALUE = 30.F;
    constexpr float MAX_VALUE = 100.F;
    constexpr float MIN_VALUE = 0.F;

    constexpr float STEP_SIZE = 1.F;
    constexpr uint8_t INIT_ENCODING = 0b00;

    // Valid quadrature transition keys.
    //
    // transition = (last_encoded << 2) | new_encoded
    //
    // encoded = (A << 1) | B
    //
    // For one direction:
    // 11 -> 01 -> 00 -> 10 -> 11
    //
    // For the other direction:
    // 11 -> 10 -> 00 -> 01 -> 11

    constexpr uint8_t CW_1 = 0b1101;
    constexpr uint8_t CW_2 = 0b0100;
    constexpr uint8_t CW_3 = 0b0010;
    constexpr uint8_t CW_4 = 0b1011;

    constexpr uint8_t CCW_1 = 0b1110;
    constexpr uint8_t CCW_2 = 0b1000;
    constexpr uint8_t CCW_3 = 0b0001;
    constexpr uint8_t CCW_4 = 0b0111;

    // 4 valid transitions per full encoder detent/click.
    constexpr int TRANSITIONS_PER_DETENT = 4;
}

struct RotaryEncoderPinout_s
{
    pin enc_switch_pin;
    pin enc_a_pin;
    pin enc_b_pin;
};

struct RotaryEncoderState_s
{
    volatile float encoder_value = default_encoder_params::INIT_ENCODER_VALUE;

    float max_value = default_encoder_params::MAX_VALUE;
    float min_value = default_encoder_params::MIN_VALUE;
    float step = default_encoder_params::STEP_SIZE;

    volatile uint8_t last_encoded = default_encoder_params::INIT_ENCODING;

    // Accumulates valid quadrature transitions.
    // +4 means one clockwise detent.
    // -4 means one counter-clockwise detent.
    volatile int transition_accumulator = 0;
};

class RotaryEncoderInterface
{
public:
    RotaryEncoderInterface(RotaryEncoderPinout_s pinout,
                        RotaryEncoderState_s state = RotaryEncoderState_s{}
    ) : _pinout(pinout),
        _state(state),
        _enc_switch_button(pinout.enc_switch_pin)
    {}

    void init();

    // Call this from the main loop.
    // Encoder movement is handled by interrupts.
    // Button debounce/state is still handled here.
    void tick(unsigned long current_millis);

    float get_value() const;

    void set_value(float value);

    void set_limits(float min_value, float max_value);

    void set_step(float step);

    bool switch_pressed();

    bool switch_released();

    bool switch_held();

private:

    RotaryEncoderPinout_s _pinout;
    RotaryEncoderState_s _state;
    ButtonInterface _enc_switch_button;

    // This implementation supports one active encoder instance.
    // That is fine for your current use case.
    static RotaryEncoderInterface* _active_instance;

    static void _isr_handler();

    uint8_t _read_encoded() const;

    void _update_encoder_from_isr();

    void _apply_transition_delta_from_isr(int delta);

    void _increment_from_isr();

    void _decrement_from_isr();

    float _clamp(float value) const;

};

using RotaryEncoderInterfaceInstance = etl::singleton<RotaryEncoderInterface>;

#endif
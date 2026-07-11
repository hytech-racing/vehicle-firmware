#include "RotaryEncoderInterface.h"


RotaryEncoderInterface* RotaryEncoderInterface::_active_instance = nullptr;

void RotaryEncoderInterface::init()
{
    pinMode(_pinout.enc_a_pin, INPUT_PULLUP);
    pinMode(_pinout.enc_b_pin, INPUT_PULLUP);

    _state.last_encoded = _read_encoded();
    _state.transition_accumulator = 0;

    _active_instance = this;

    attachInterrupt(
        digitalPinToInterrupt(_pinout.enc_a_pin),
        RotaryEncoderInterface::_isr_handler,
        CHANGE
    );

    attachInterrupt(
        digitalPinToInterrupt(_pinout.enc_b_pin),
        RotaryEncoderInterface::_isr_handler,
        CHANGE
    );
}

void RotaryEncoderInterface::tick(unsigned long current_millis)
{
    // Do not update encoder here anymore.
    // Encoder is updated by interrupt.
    _enc_switch_button.update(current_millis);
}

float RotaryEncoderInterface::get_value() const
{
    noInterrupts();
    float value = _state.encoder_value;
    interrupts();

    return value;
}

void RotaryEncoderInterface::set_value(float value)
{
    noInterrupts();
    _state.encoder_value = _clamp(value);
    _state.transition_accumulator = 0;
    interrupts();
}

void RotaryEncoderInterface::set_limits(float min_value, float max_value)
{
    if (min_value > max_value)
    {
        float temp = min_value;
        min_value = max_value;
        max_value = temp;
    }

    noInterrupts();

    _state.min_value = min_value;
    _state.max_value = max_value;
    _state.encoder_value = _clamp(_state.encoder_value);
    _state.transition_accumulator = 0;

    interrupts();
}

void RotaryEncoderInterface::set_step(float step)
{
    if (step < 0)
    {
        step = -step;
    }

    noInterrupts();
    _state.step = step;
    interrupts();
}

bool RotaryEncoderInterface::switch_pressed()
{
    return _enc_switch_button.is_pressed();
}

bool RotaryEncoderInterface::switch_released()
{
    return _enc_switch_button.is_released();
}

bool RotaryEncoderInterface::switch_held()
{
    return _enc_switch_button.is_held();
}

void RotaryEncoderInterface::_isr_handler()
{
    if (_active_instance != nullptr)
    {
        _active_instance->_update_encoder_from_isr();
    }
}

uint8_t RotaryEncoderInterface::_read_encoded() const
{
    uint8_t a = static_cast<uint8_t>(digitalRead(_pinout.enc_a_pin));
    uint8_t b = static_cast<uint8_t>(digitalRead(_pinout.enc_b_pin));

    return static_cast<uint8_t>((a << 1) | b);
}

void RotaryEncoderInterface::_update_encoder_from_isr()
{
    uint8_t encoded = _read_encoded();

    if (encoded == _state.last_encoded)
    {
        return;
    }

    uint8_t transition = static_cast<uint8_t>((_state.last_encoded << 2) | encoded);

    switch (transition)
    {
        case default_encoder_params::CW_1:
        case default_encoder_params::CW_2:
        case default_encoder_params::CW_3:
        case default_encoder_params::CW_4:
        {
            _apply_transition_delta_from_isr(+1);
            break;
        }

        case default_encoder_params::CCW_1:
        case default_encoder_params::CCW_2:
        case default_encoder_params::CCW_3:
        case default_encoder_params::CCW_4:
        {
            _apply_transition_delta_from_isr(-1);
            break;
        }

        default:
        {
            // Invalid transition.
            // This can happen from switch bounce or missed edges.
            // Resetting the accumulator avoids applying a fake detent.
            _state.transition_accumulator = 0;
            break;
        }
    }

    _state.last_encoded = encoded;
}

void RotaryEncoderInterface::_apply_transition_delta_from_isr(int delta)
{
    _state.transition_accumulator += delta;

    if (_state.transition_accumulator >= default_encoder_params::TRANSITIONS_PER_DETENT)
    {
        _increment_from_isr();
        _state.transition_accumulator = 0;
    }
    else if (_state.transition_accumulator <= -default_encoder_params::TRANSITIONS_PER_DETENT)
    {
        _decrement_from_isr();
        _state.transition_accumulator = 0;
    }
}

void RotaryEncoderInterface::_increment_from_isr()
{
    float next_value = _state.encoder_value + _state.step;
    _state.encoder_value = _clamp(next_value);
}

void RotaryEncoderInterface::_decrement_from_isr()
{
    float next_value = _state.encoder_value - _state.step;
    _state.encoder_value = _clamp(next_value);
}

float RotaryEncoderInterface::_clamp(float value) const
{
    if (value > _state.max_value)
    {
        return _state.max_value;
    }

    if (value < _state.min_value)
    {
        return _state.min_value;
    }

    return value;
}
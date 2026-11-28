#include "ADS112Interface.hpp"

ADS112Interface::ADS112Interface(
    HardwareSerial& serial,
    const std::array<float, NUM_CHANNELS>& scales,
    const std::array<float, NUM_CHANNELS>& offsets,
    ADS112Pinout_s pinouts,
    ADS112Config_s config,
    ADS112Commands_s commands,
    ADS112Timing_s timing,
    uint32_t baud_rate)
    : _serial(&serial), _pinouts(pinouts), _config(config), _commands(commands), _timing(timing), _baud_rate(baud_rate)
{
    for (int ch = 0; ch < NUM_CHANNELS; ch++)
    {
        this->_channels[ch] = AnalogChannel();
        this->setChannelScaleAndOffset(ch, scales[ch], offsets[ch]);
    }
}

void ADS112Interface::init()
{
    if (_hasResetPin())
    {
        pinMode(_pinouts.reset_pin, OUTPUT);
        digitalWrite(_pinouts.reset_pin, HIGH);
    }

    if (_hasDrdyPin())
    {
        pinMode(_pinouts.drdy_pin, INPUT);
    }

    _serial->begin(_baud_rate);

    // Let the UART peripheral and ADC power-up fully before we transmit.
    delay(5);
    _clearRx();

    // Kick off the initialisation state machine.
    _enterState(ADS112State_e::INIT_RESET);
}

void ADS112Interface::tick()
{
    const uint32_t now = millis();

    switch (_state)
    {
        // -----------------------------------------------------------------------
        // Initialisation sequence
        // -----------------------------------------------------------------------

    case ADS112State_e::INIT_RESET:
        _sendCommand(_commands.reset_command);
        _enterState(ADS112State_e::INIT_WAIT_RESET);
        break;

    case ADS112State_e::INIT_WAIT_RESET:
        // Wait for reset delay before touching the ADC again.
        if ((now - _state_entered_ms) >= _timing.reset_delay_ms)
        {
            _clearRx();
            _enterState(ADS112State_e::INIT_WRITE_REG0);
        }
        break;

    case ADS112State_e::INIT_WRITE_REG0:
        // MUX for channel 0, gain, PGA bypass.
        _writeRegister(0x00, _buildConfigRegister0(_config.muxes.at(0)));
        _enterState(ADS112State_e::INIT_WRITE_REG1);
        break;

    case ADS112State_e::INIT_WRITE_REG1:
        // Data rate, mode, single-shot, voltage reference.
        _writeRegister(0x01, _buildConfigRegister1());
        _enterState(ADS112State_e::INIT_WRITE_REG2);
        break;

    case ADS112State_e::INIT_WRITE_REG2:
        // No data counter, no CRC, no burnout, no IDAC.
        _writeRegister(0x02, 0x00);
        _enterState(ADS112State_e::INIT_WRITE_REG3);
        break;

    case ADS112State_e::INIT_WRITE_REG3:
        // No IDACs, manual data read mode (AUTO=0).
        _writeRegister(0x03, 0x00);
        _enterState(ADS112State_e::INIT_WRITE_REG4);
        break;

    case ADS112State_e::INIT_WRITE_REG4:
        // GPIO2/DRDY configured as DRDY output.
        _writeRegister(0x04, _config.config_reg_4_default_value);
        _current_channel = 0;
        _enterState(ADS112State_e::IDLE);
        break;

        // -----------------------------------------------------------------------
        // Normal sampling sequence
        // -----------------------------------------------------------------------

    case ADS112State_e::IDLE:
        // Update MUX for the current channel then immediately start.
        _writeRegister(0x00, _buildConfigRegister0(_config.muxes.at(_current_channel)));
        _enterState(ADS112State_e::START_CONVERSION);
        break;

    case ADS112State_e::START_CONVERSION:
        _clearRx();
        _sendCommand(_commands.start_sync_command);
        _enterState(ADS112State_e::WAIT_CONVERSION);
        break;

    case ADS112State_e::WAIT_CONVERSION:
        // Non-blocking wait — come back next tick until the time has elapsed.
        if ((now - _state_entered_ms) >= _timing.conversion_wait_ms)
        {
            _enterState(ADS112State_e::REQUEST_DATA);
        }
        break;

    case ADS112State_e::REQUEST_DATA:
        _clearRx();
        _sendCommand(_commands.rdata_command);
        _timeout_start_ms = millis();
        _enterState(ADS112State_e::READ_DATA);
        break;

    case ADS112State_e::READ_DATA:
        // Wait until 2 bytes have arrived or we time out.
        if (_serial->available() >= 2)
        {
            _enterState(ADS112State_e::PROCESS_DATA);
        }
        else if ((millis() - _timeout_start_ms) > _timing.read_timeout_ms)
        {
            // Timeout — skip this channel and move on.
            _current_channel = (_current_channel + 1) % NUM_CHANNELS;
            _enterState(ADS112State_e::IDLE);
        }
        break;

    case ADS112State_e::PROCESS_DATA:
    {
        // Drain any extra bytes, keeping only the last 2.
        delayMicroseconds(20);
        while (_serial->available() > 2)
            _serial->read();

        const uint8_t lsb = static_cast<uint8_t>(_serial->read());
        const uint8_t msb = static_cast<uint8_t>(_serial->read());

        const int16_t raw = static_cast<int16_t>((static_cast<uint16_t>(msb) << 8U) | lsb);

        this->_channels[_current_channel].lastSample = raw;

        // Convert the freshly-sampled channel immediately.
        this->_convert();

        // Advance to the next channel.
        _current_channel = (_current_channel + 1) % NUM_CHANNELS;
        _enterState(ADS112State_e::IDLE);
        break;
    }

    case ADS112State_e::FAULT:
        // Nothing to do — caller can check isFaulted().
        break;
    }
}

AnalogConversion_s ADS112Interface::getChannel(size_t channel)
{
    return this->data.conversions.at(channel);
}

bool ADS112Interface::isReady() const
{
    return _state == ADS112State_e::IDLE
           || _state == ADS112State_e::START_CONVERSION
           || _state == ADS112State_e::WAIT_CONVERSION
           || _state == ADS112State_e::REQUEST_DATA
           || _state == ADS112State_e::READ_DATA
           || _state == ADS112State_e::PROCESS_DATA;
}

bool ADS112Interface::isFaulted() const
{
    return _state == ADS112State_e::FAULT;
}

ADS112State_e ADS112Interface::getState() const
{
    return _state;
}

void ADS112Interface::powerDown()
{
    _sendCommand(_commands.powerdown_command);
}

ADS112TemperatureReading_s ADS112Interface::getInternalTemperature()
{
    return _internal_temperature;
}

void ADS112Interface::_sendByte(uint8_t b)
{
    _serial->write(b);
    _serial->flush();
    delayMicroseconds(_timing.byte_gap_us);
}

void ADS112Interface::_sendCommand(uint8_t command)
{
    _sendByte(_commands.sync_word);
    _sendByte(command);
}

void ADS112Interface::_writeRegister(uint8_t register_address, uint8_t value)
{
    const uint8_t command = static_cast<uint8_t>(_commands.wreg_base_command | ((register_address & 0x07U) << 1U));
    _sendByte(_commands.sync_word);
    _sendByte(command);
    _sendByte(value);
}

void ADS112Interface::_clearRx()
{
    delayMicroseconds(5); // let any in-flight bytes arrive
    while (_serial->available())
        _serial->read();
}

void ADS112Interface::_enterState(ADS112State_e next)
{
    _state = next;
    _state_entered_ms = millis();
}

uint8_t ADS112Interface::_buildConfigRegister0(ADS112U04InputMux_e mux) const
{
    const uint8_t mux_bits = static_cast<uint8_t>(mux);
    const uint8_t gain_bits = static_cast<uint8_t>(_config.gain);

    // Single-ended MUX settings (AINx vs AVSS) require PGA bypass.
    constexpr uint8_t PGA_BYPASS_ENABLED = 0x01U;

    return static_cast<uint8_t>((mux_bits << 4U) | (gain_bits << 1U) | PGA_BYPASS_ENABLED);
}

uint8_t ADS112Interface::_buildConfigRegister1(bool temperature_sensor_enabled) const
{
    const uint8_t data_rate_bits = static_cast<uint8_t>(_config.data_rate);
    const uint8_t reference_bits = static_cast<uint8_t>(_config.reference);

    constexpr uint8_t NORMAL_MODE = 0x00U;
    constexpr uint8_t SINGLE_SHOT = 0x00U;
    const uint8_t temp_sensor = temperature_sensor_enabled ? 0x01U : 0x00U;

    return static_cast<uint8_t>(
        (data_rate_bits << ads112_bit_shift_params::DATA_RATE_SHIFT)
        | (NORMAL_MODE << ads112_bit_shift_params::NORMAL_MODE_SHIFT)
        | (SINGLE_SHOT << ads112_bit_shift_params::SINGLE_SHOT_SHIFT)
        | (reference_bits << ads112_bit_shift_params::REFERENCE_SHIFT)
        | temp_sensor);
}

bool ADS112Interface::_hasResetPin() const
{
    return _pinouts.reset_pin >= 0;
}

bool ADS112Interface::_hasDrdyPin() const
{
    return _pinouts.drdy_pin >= 0;
}
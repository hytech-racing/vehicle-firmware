#include "OrbisInterface.h"


OrbisInterface::OrbisInterface(HardwareSerial* serial) : _serial(serial)
{
   _lastReading.status = SteeringEncoderStatus_e::ERROR;
   _serial->begin(orbis_constants::DEFAULT_BAUD_RATE, SERIAL_8N1);
}

bool OrbisInterface::performSelfCalibration()
{
    _orbisErrors.calibration_timeout   = false;
    _orbisErrors.calibration_parameter = false;

    /**
     * Datasheet recommends doing status before and after self-calibration command.
     * Status command returns 2 bytes: echo byte + status byte
     */

    /* ----- SELF CALIBRATION STATUS 1 ----- */
    _serial->write(orbis_commands::SELF_CALIB_STATUS); delay(1);

    uint8_t echo1 = _serial->read();
    uint8_t status_before_calib = _serial->read();

    uint8_t calibration_counter_before = status_before_calib & orbis_bitmasks::SELF_CALIB_STATUS_BITMASK;


    /* ----- SELF CALIBRATION START ----- */
    _sendUnlockSequence();

    _serial->write(orbis_commands::SELF_CALIB_START); delay(1);

    while (!_serial->available());  // wait until bytes are available
    uint8_t echo2 = _serial->read();

    // Serial.println("ROTATE NOW");      // Debug line

    delay(orbis_constants::SELF_CALIB_MAX_TIME_MS); // Max time for a self-calibration is 10 seconds.


    /* ----- SELF CALIBRATION STATUS 2 ----- */
    _serial->write(orbis_commands::SELF_CALIB_STATUS); delay(1);

    unsigned long startTime = millis();
    while (_serial->available() < 2)
    {
        if (millis() - startTime >= orbis_constants::TIMEOUT_MS)
        {
            return false;  // timeout
        }
    }
    // This is an additional check to ensure that we receive the expected two bytes. If we don't, we want to end calibration
    // and know that an error occurred.

    uint8_t echo = _serial->read();
    uint8_t status_after_calib = _serial->read();

    uint8_t calibration_counter_after = status_after_calib & orbis_bitmasks::SELF_CALIB_NEW_COUNTER_BITMASK;
    // Encoder increments a 2-bit counter each successful calibration.
    // We verify calibration completed by confirming this counter changed.

    bool timeout_error = status_after_calib & orbis_bitmasks::SELF_CALIB_TIMEOUT_ERROR_BITMASK;      // b2
    bool parameter_error = status_after_calib & orbis_bitmasks::SELF_CALIB_PARAMETER_ERROR_BITMASK;  // b3

    if (calibration_counter_before == calibration_counter_after) {
        // Serial.println("ERROR: Calibration Did Not Complete");  // Debug Line
        return false;
    }

    if (timeout_error || parameter_error)
    {
        _orbisErrors.calibration_timeout   = timeout_error;
        _orbisErrors.calibration_parameter = parameter_error;
        return false;
    }

    return true;
}

void OrbisInterface::setEncoderOffset()
{
    factoryReset();

    _serial->write(orbis_commands::SHORT_POS_REQUEST); delay(1);

    while (!_serial->available());  // wait until bytes are available
    uint8_t position_high_byte = _serial->read();
    while (!_serial->available());  // wait until bytes are available
    uint8_t position_low_byte = _serial->read();

    uint16_t current_raw_position = ((((uint16_t) position_high_byte) << orbis_constants::POSITION_DATA_HIGH_BYTE_SHIFT) | (uint16_t) position_low_byte) >> orbis_constants::POSITION_DATA_RIGHT_SHIFT;
    byte offset_pos_high_byte = (current_raw_position >> orbis_constants::OFFSET_HIGH_BYTE_SHIFT) & orbis_bitmasks::OFFSET_RECOMBINING_MASK;   // b15 - b8.
    byte offset_pos_low_byte = current_raw_position & orbis_bitmasks::OFFSET_RECOMBINING_MASK;    // b7 - b0
    // Current raw position is 14 bit integer, we parse two bytes to remove warning/error bits.
    // Offset command only takes in position data as individual bytes (8 bit integer). Break 14 bit integer into two bytes.

    _sendUnlockSequence();

    _serial->write(orbis_commands::POSITION_OFFSET); delay(1);
    _serial->write((byte) 0x00); delay(1);
    _serial->write((byte) 0x00); delay(1);
    _serial->write(offset_pos_high_byte); delay(1);
    _serial->write(offset_pos_low_byte); delay(1);
    // After writing POSITION_OFFSET, the next four bytes are what you subtract from your initial position.
    // We want to start at 0 degrees, so subtract out initial position. Inital position - inital position = 0.

    _flushSerialBuffer();
    saveConfiguration();
}

void OrbisInterface::saveConfiguration()
{
    _sendUnlockSequence();

    _serial->write(orbis_commands::SAVE_CONFIG); delay(1);

    _flushSerialBuffer();

    delay(orbis_constants::SAVE_CONFIG_DELAY_MS);
}



/* -------------------- Sample/Data Methods -------------------- */
void OrbisInterface::sample()
{

    _lastReading.errors = EncoderErrorFlags_s{};  // reset to all false
    _orbisErrors        = OrbisErrorFlags_s{};    // reset to all false

    _serial->write(orbis_commands::DETAILED_POS_REQUEST); delay(1);
    // Detailed Position Request: 1 byte echo, 2 byte position, 1 byte detailed status

    unsigned long startTime = millis();
    while (_serial->available() < 4)
    {
        if (millis() - startTime >= orbis_constants::TIMEOUT_MS) // NOLINT
        {
            _lastReading.errors.noData = true;
            _lastReading.status = SteeringEncoderStatus_e::ERROR;
            return;
        }
    }
    // Check to ensure that we receive the expected 4 bytes. If we don't, communication is erroring and we want to flag it.

    uint8_t echo = _serial->read();

    if (echo != orbis_commands::DETAILED_POS_REQUEST)
    {
        _lastReading.errors.noData = true;
        _lastReading.status = SteeringEncoderStatus_e::ERROR;
        return;
    }

    uint8_t general_high_byte = _serial->read();
    uint8_t general_low_byte = _serial->read();
    uint8_t detailed = _serial->read();
    uint8_t general_status = general_low_byte & orbis_bitmasks::SELF_CALIB_STATUS_BITMASK;  // b1:b0 are general status

    _decodeErrors(general_status, detailed);
    // Decode errors using the extracted general status bits + detailed errors byte

    uint16_t raw_position = ((((uint16_t) general_high_byte) << orbis_constants::POSITION_DATA_HIGH_BYTE_SHIFT) | (uint16_t) general_low_byte) >> orbis_constants::POSITION_DATA_RIGHT_SHIFT;
    _lastReading.rawValue = raw_position;

    float angle = ((float)raw_position / orbis_constants::ENCODER_RESOLUTION ) * orbis_constants::DEGREES_PER_REVOLUTION;
    if (angle > orbis_constants::ANGLE_WRAPAROUND_THRESHOLD)
    {
        angle -= orbis_constants::FULL_ROTATION_DEGREES;
    }

    _lastReading.angle = angle;
    // "raw_position" / "ENCODER_RESOLUTION" = fraction of one full rotation. Then multiple by "DEGREES_PER_REVOLUTION" (360), to get the exact angle.
    // We want the sensor to work such that the middle is 0, left goes to -180 degrees, and right goes to +180 degrees.
    // Thus, if the angle is > 180 (ANGLE_CONVERSION_MARKER), we want to subtract 360 (ANGLE_CONVERSION) so that it's negative.

    // Serial.print("Raw ");
    // Serial.println(_lastReading.rawValue, HEX);
    // Serial.print("Angle ");
    // Serial.println(_lastReading.angle);
}

SteeringEncoderReading_s OrbisInterface::getLastReading()
{
    return _lastReading;
}

/* -------------------- Error Flagging -------------------- */
void OrbisInterface::_decodeErrors(uint8_t general, uint8_t detailed)
{
    // General bits error low (0)
    _lastReading.errors.dataInvalid      = !(general & orbis_bitmasks::GENERAL_ERROR_BITMASK);
    _lastReading.errors.operatingLimit   = !(general & orbis_bitmasks::GENERAL_WARNING_BITMASK);

    // Detailed bits are error high (1)
    _orbisErrors.counter_error      = detailed & orbis_bitmasks::DETAILED_COUNTER_ERROR_BITMASK;
    _orbisErrors.speed_high         = detailed & orbis_bitmasks::DETAILED_SPEED_HIGH_BITMASK;
    _orbisErrors.temp_out_of_range  = detailed & orbis_bitmasks::DETAILED_TEMP_RANGE_BITMASK;
    _orbisErrors.dist_far           = detailed & orbis_bitmasks::DETAILED_DIST_FAR_BITMASK;
    _orbisErrors.dist_near          = detailed & orbis_bitmasks::DETAILED_DIST_NEAR_BITMASK;

    bool anyError =
    (
        _lastReading.errors.dataInvalid  ||
        _lastReading.errors.noData       ||
        _orbisErrors.counter_error       ||
        _orbisErrors.speed_high          ||
        _orbisErrors.temp_out_of_range   ||
        _orbisErrors.dist_far            ||
        _orbisErrors.dist_near
    );

    _lastReading.status = anyError ? SteeringEncoderStatus_e::ERROR : SteeringEncoderStatus_e::NOMINAL;
}

void OrbisInterface::factoryReset()
{
    _sendUnlockSequence();
    _serial->write(orbis_commands::FACTORY_RESET);
    _flushSerialBuffer();
    delay(orbis_constants::FACTORY_RESET_DELAY_MS);
}

void OrbisInterface::_sendUnlockSequence()
{
    for (byte b : orbis_commands::UNLOCK_SEQUENCE) {
        _serial->write(b); delay(1);
        _serial->read();
    }
}

void OrbisInterface::_flushSerialBuffer()
{
    while (_serial->available())
    {
        _serial->read();
    }
}

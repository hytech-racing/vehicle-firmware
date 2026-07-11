#include "BrakeRotorTempInterface.h"
#include "hytech.h"

BrakeTempData_s BrakeRotorTempInterface::getBrakeRotorTempData() const {
    return _temp_data;
}

void BrakeRotorTempInterface::receiveBrakeRotorTempData(const CAN_message_t &msg) {
    switch (msg.id) {
        case FL_BRAKE_ROTOR_TEMP_CH1_CH4_CANID:
        {
            // unpack the msg
            FL_BRAKE_ROTOR_TEMP_CH1_CH4_t unpacked_msg;
            Unpack_FL_BRAKE_ROTOR_TEMP_CH1_CH4_hytech(&unpacked_msg, msg.buf, msg.len); // NOLINT array decay to pointer

            // copy data over to interface data
            std::get<0>(_temp_data.fl_sensor.channel_data) = HYTECH_brake_temp_channel_1_ro_fromS(unpacked_msg.brake_temp_channel_1_ro);
            std::get<1>(_temp_data.fl_sensor.channel_data) = HYTECH_brake_temp_channel_2_ro_fromS(unpacked_msg.brake_temp_channel_2_ro);
            std::get<2>(_temp_data.fl_sensor.channel_data) = HYTECH_brake_temp_channel_3_ro_fromS(unpacked_msg.brake_temp_channel_3_ro);
            std::get<3>(_temp_data.fl_sensor.channel_data) = HYTECH_brake_temp_channel_4_ro_fromS(unpacked_msg.brake_temp_channel_4_ro);

            // update FL outputs
            _updateCalculatedValues(false);
            break;
        }

        case FL_BRAKE_ROTOR_TEMP_CH5_CH8_CANID:
        {
            // unpack the msg
            FL_BRAKE_ROTOR_TEMP_CH5_CH8_t unpacked_msg;
            Unpack_FL_BRAKE_ROTOR_TEMP_CH5_CH8_hytech(&unpacked_msg, msg.buf, msg.len); // NOLINT array decay to pointer

            // copy data over to interface data
            std::get<4>(_temp_data.fl_sensor.channel_data) = HYTECH_brake_temp_channel_5_ro_fromS(unpacked_msg.brake_temp_channel_5_ro);
            std::get<5>(_temp_data.fl_sensor.channel_data) = HYTECH_brake_temp_channel_6_ro_fromS(unpacked_msg.brake_temp_channel_6_ro);
            std::get<6>(_temp_data.fl_sensor.channel_data) = HYTECH_brake_temp_channel_7_ro_fromS(unpacked_msg.brake_temp_channel_7_ro);
            std::get<7>(_temp_data.fl_sensor.channel_data) = HYTECH_brake_temp_channel_8_ro_fromS(unpacked_msg.brake_temp_channel_8_ro);

            // update FL outputs
            _updateCalculatedValues(false);
            break;
        }

        case FL_BRAKE_ROTOR_TEMP_CH9_CH12_CANID:
        {
            // unpack the msg
            FL_BRAKE_ROTOR_TEMP_CH9_CH12_t unpacked_msg;
            Unpack_FL_BRAKE_ROTOR_TEMP_CH9_CH12_hytech(&unpacked_msg, msg.buf, msg.len); // NOLINT array decay to pointer

            // copy data over to interface data
            std::get<8>(_temp_data.fl_sensor.channel_data) = HYTECH_brake_temp_channel_9_ro_fromS(unpacked_msg.brake_temp_channel_9_ro);
            std::get<9>(_temp_data.fl_sensor.channel_data) = HYTECH_brake_temp_channel_10_ro_fromS(unpacked_msg.brake_temp_channel_10_ro);
            std::get<10>(_temp_data.fl_sensor.channel_data) = HYTECH_brake_temp_channel_11_ro_fromS(unpacked_msg.brake_temp_channel_11_ro);
            std::get<11>(_temp_data.fl_sensor.channel_data) = HYTECH_brake_temp_channel_12_ro_fromS(unpacked_msg.brake_temp_channel_12_ro);

            // update FL outputs
            _updateCalculatedValues(false);
            break;
        }

        case FL_BRAKE_ROTOR_TEMP_CH13_CH16_CANID:
        {
            // unpack the msg
            FL_BRAKE_ROTOR_TEMP_CH13_CH16_t unpacked_msg;
            Unpack_FL_BRAKE_ROTOR_TEMP_CH13_CH16_hytech(&unpacked_msg, msg.buf, msg.len); // NOLINT array decay to pointer

            // copy data over to interface data
            std::get<12>(_temp_data.fl_sensor.channel_data) = HYTECH_brake_temp_channel_13_ro_fromS(unpacked_msg.brake_temp_channel_13_ro);
            std::get<13>(_temp_data.fl_sensor.channel_data) = HYTECH_brake_temp_channel_14_ro_fromS(unpacked_msg.brake_temp_channel_14_ro);
            std::get<14>(_temp_data.fl_sensor.channel_data) = HYTECH_brake_temp_channel_15_ro_fromS(unpacked_msg.brake_temp_channel_15_ro);
            std::get<15>(_temp_data.fl_sensor.channel_data) = HYTECH_brake_temp_channel_16_ro_fromS(unpacked_msg.brake_temp_channel_16_ro);

            // update FL outputs
            _updateCalculatedValues(false);
            break;
        }

        case FR_BRAKE_ROTOR_TEMP_CH1_CH4_CANID:
        {
            // unpack the msg
            FR_BRAKE_ROTOR_TEMP_CH1_CH4_t unpacked_msg;
            Unpack_FR_BRAKE_ROTOR_TEMP_CH1_CH4_hytech(&unpacked_msg, msg.buf, msg.len); // NOLINT array decay to pointer

            // copy data over to interface data
            std::get<0>(_temp_data.fr_sensor.channel_data) = HYTECH_brake_temp_channel_1_ro_fromS(unpacked_msg.brake_temp_channel_1_ro);
            std::get<1>(_temp_data.fr_sensor.channel_data) = HYTECH_brake_temp_channel_2_ro_fromS(unpacked_msg.brake_temp_channel_2_ro);
            std::get<2>(_temp_data.fr_sensor.channel_data) = HYTECH_brake_temp_channel_3_ro_fromS(unpacked_msg.brake_temp_channel_3_ro);
            std::get<3>(_temp_data.fr_sensor.channel_data) = HYTECH_brake_temp_channel_4_ro_fromS(unpacked_msg.brake_temp_channel_4_ro);

            // update FR outputs
            _updateCalculatedValues(true);
            break;
        }

        case FR_BRAKE_ROTOR_TEMP_CH5_CH8_CANID:
        {
            // unpack the msg
            FR_BRAKE_ROTOR_TEMP_CH5_CH8_t unpacked_msg;
            Unpack_FR_BRAKE_ROTOR_TEMP_CH5_CH8_hytech(&unpacked_msg, msg.buf, msg.len); // NOLINT array decay to pointer

            // copy data over to interface data
            std::get<4>(_temp_data.fr_sensor.channel_data) = HYTECH_brake_temp_channel_5_ro_fromS(unpacked_msg.brake_temp_channel_5_ro);
            std::get<5>(_temp_data.fr_sensor.channel_data) = HYTECH_brake_temp_channel_6_ro_fromS(unpacked_msg.brake_temp_channel_6_ro);
            std::get<6>(_temp_data.fr_sensor.channel_data) = HYTECH_brake_temp_channel_7_ro_fromS(unpacked_msg.brake_temp_channel_7_ro);
            std::get<7>(_temp_data.fr_sensor.channel_data) = HYTECH_brake_temp_channel_8_ro_fromS(unpacked_msg.brake_temp_channel_8_ro);

            // update FR outputs
            _updateCalculatedValues(true);
            break;
        }

        case FR_BRAKE_ROTOR_TEMP_CH9_CH12_CANID:
        {
            // unpack the msg
            FR_BRAKE_ROTOR_TEMP_CH9_CH12_t unpacked_msg;
            Unpack_FR_BRAKE_ROTOR_TEMP_CH9_CH12_hytech(&unpacked_msg, msg.buf, msg.len); // NOLINT array decay to pointer

            // copy data over to interface data
            std::get<8>(_temp_data.fr_sensor.channel_data) = HYTECH_brake_temp_channel_9_ro_fromS(unpacked_msg.brake_temp_channel_9_ro);
            std::get<9>(_temp_data.fr_sensor.channel_data) = HYTECH_brake_temp_channel_10_ro_fromS(unpacked_msg.brake_temp_channel_10_ro);
            std::get<10>(_temp_data.fr_sensor.channel_data) = HYTECH_brake_temp_channel_11_ro_fromS(unpacked_msg.brake_temp_channel_11_ro);
            std::get<11>(_temp_data.fr_sensor.channel_data) = HYTECH_brake_temp_channel_12_ro_fromS(unpacked_msg.brake_temp_channel_12_ro);

            // update FR outputs
            _updateCalculatedValues(true);
            break;
        }

        case FR_BRAKE_ROTOR_TEMP_CH13_CH16_CANID:
        {
            // unpack the msg
            FR_BRAKE_ROTOR_TEMP_CH13_CH16_t unpacked_msg;
            Unpack_FR_BRAKE_ROTOR_TEMP_CH13_CH16_hytech(&unpacked_msg, msg.buf, msg.len); // NOLINT array decay to pointer

            // copy data over to interface data
            std::get<12>(_temp_data.fr_sensor.channel_data) = HYTECH_brake_temp_channel_13_ro_fromS(unpacked_msg.brake_temp_channel_13_ro);
            std::get<13>(_temp_data.fr_sensor.channel_data) = HYTECH_brake_temp_channel_14_ro_fromS(unpacked_msg.brake_temp_channel_14_ro);
            std::get<14>(_temp_data.fr_sensor.channel_data) = HYTECH_brake_temp_channel_15_ro_fromS(unpacked_msg.brake_temp_channel_15_ro);
            std::get<15>(_temp_data.fr_sensor.channel_data) = HYTECH_brake_temp_channel_16_ro_fromS(unpacked_msg.brake_temp_channel_16_ro);

            // update FR outputs
            _updateCalculatedValues(true);
            break;
        }

        default:
        break;
    }
}

/**
 * Helper method to update the calculated values of max and avg temps for each sensor after new data is
 * received. Will only update for the specified sensor.
 * @param sensor corresponds to which sensor was updated. FL = 0, FR = 1
 */
void BrakeRotorTempInterface::_updateCalculatedValues(bool FR) {
    if (FR) { // check if FR needs to be updated
        auto begin_iterator = _temp_data.fr_sensor.channel_data.begin();
        auto end_iterator = _temp_data.fr_sensor.channel_data.end();

        // update maximum value
        _temp_data.fr_sensor.max_temp = *std::max_element(begin_iterator, end_iterator);

        // update avg value
        float sum = std::accumulate(begin_iterator, end_iterator, 0.0f); // find sum
        _temp_data.fr_sensor.avg_temp = sum / static_cast<float>(brake_rotor_temp_default_params::channels_within_brake_temp_sensor);
    } else { // otherwise update FL
        auto begin_iterator = _temp_data.fl_sensor.channel_data.begin();
        auto end_iterator = _temp_data.fl_sensor.channel_data.end();

        // update maximum value
        _temp_data.fl_sensor.max_temp = *std::max_element(begin_iterator, end_iterator);

        // update avg value
        float sum = std::accumulate(begin_iterator, end_iterator, 0.0f); // find sum
        _temp_data.fl_sensor.avg_temp = sum / static_cast<float>(brake_rotor_temp_default_params::channels_within_brake_temp_sensor);
    }
}
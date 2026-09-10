#include "TTPMSInterface.h"
#include "VCRCANInterfaceImpl.h"


TTPMSAllSensorData_s TTPMSInterface::getTTPMSData() const
{
    return _ttpms_data;
}


void TTPMSInterface::receiveTTPMSData(const CAN_message_t &can_msg)
{
    switch (can_msg.id)
    {
        // Left front TTPMS
        {
            case LF_TTPMS_1_CANID:
            {
                LF_TTPMS_1_t unpacked_can_msg;
                Unpack_LF_TTPMS_1_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_LF_TTPMS_1_hytech,
                                    VCRCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.fl_ttpms.serial_number = unpacked_can_msg.LF_TTPMS_SN;
                _ttpms_data.fl_ttpms.bat_voltage = unpacked_can_msg.LF_TTPMS_BAT_V;
                _ttpms_data.fl_ttpms.pressure = HYTECH_LF_TTPMS_P_ro_fromS(unpacked_can_msg.LF_TTPMS_P_ro);
                _ttpms_data.fl_ttpms.gauge_pressure = unpacked_can_msg.LF_TTPMS_P_GAUGE;

                break;
            }
            case LF_TTPMS_2_CANID:
            {
                LF_TTPMS_2_t unpacked_can_msg;
                Unpack_LF_TTPMS_2_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_LF_TTPMS_2_hytech,
                                    VCRCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.fl_ttpms.temp_data[0] = HYTECH_LF_TTPMS_T1_ro_fromS(unpacked_can_msg.LF_TTPMS_T1_ro);
                _ttpms_data.fl_ttpms.temp_data[1] = HYTECH_LF_TTPMS_T2_ro_fromS(unpacked_can_msg.LF_TTPMS_T2_ro);
                _ttpms_data.fl_ttpms.temp_data[2] = HYTECH_LF_TTPMS_T3_ro_fromS(unpacked_can_msg.LF_TTPMS_T3_ro);
                _ttpms_data.fl_ttpms.temp_data[3] = HYTECH_LF_TTPMS_T4_ro_fromS(unpacked_can_msg.LF_TTPMS_T4_ro);

                break;
            }
            case LF_TTPMS_3_CANID:
            {
                LF_TTPMS_3_t unpacked_can_msg;
                Unpack_LF_TTPMS_3_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_LF_TTPMS_3_hytech,
                                    VCRCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.fl_ttpms.temp_data[4] = HYTECH_LF_TTPMS_T5_ro_fromS(unpacked_can_msg.LF_TTPMS_T5_ro);
                _ttpms_data.fl_ttpms.temp_data[5] = HYTECH_LF_TTPMS_T6_ro_fromS(unpacked_can_msg.LF_TTPMS_T6_ro);
                _ttpms_data.fl_ttpms.temp_data[6] = HYTECH_LF_TTPMS_T7_ro_fromS(unpacked_can_msg.LF_TTPMS_T7_ro);
                _ttpms_data.fl_ttpms.temp_data[7] = HYTECH_LF_TTPMS_T8_ro_fromS(unpacked_can_msg.LF_TTPMS_T8_ro);
                break;
            }
            case LF_TTPMS_4_CANID:
            {
                LF_TTPMS_4_t unpacked_can_msg;
                Unpack_LF_TTPMS_4_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_LF_TTPMS_4_hytech,
                                    VCRCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.fl_ttpms.temp_data[8] = HYTECH_LF_TTPMS_T9_ro_fromS(unpacked_can_msg.LF_TTPMS_T9_ro);
                _ttpms_data.fl_ttpms.temp_data[9] = HYTECH_LF_TTPMS_T10_ro_fromS(unpacked_can_msg.LF_TTPMS_T10_ro);
                _ttpms_data.fl_ttpms.temp_data[10] = HYTECH_LF_TTPMS_T11_ro_fromS(unpacked_can_msg.LF_TTPMS_T11_ro);
                _ttpms_data.fl_ttpms.temp_data[11] = HYTECH_LF_TTPMS_T12_ro_fromS(unpacked_can_msg.LF_TTPMS_T12_ro);

                break;
            }
            case LF_TTPMS_5_CANID:
            {
                LF_TTPMS_5_t unpacked_can_msg;
                Unpack_LF_TTPMS_5_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_LF_TTPMS_5_hytech,
                                    VCRCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.fl_ttpms.temp_data[12] = HYTECH_LF_TTPMS_T13_ro_fromS(unpacked_can_msg.LF_TTPMS_T13_ro);
                _ttpms_data.fl_ttpms.temp_data[13] = HYTECH_LF_TTPMS_T14_ro_fromS(unpacked_can_msg.LF_TTPMS_T14_ro);
                _ttpms_data.fl_ttpms.temp_data[14] = HYTECH_LF_TTPMS_T15_ro_fromS(unpacked_can_msg.LF_TTPMS_T15_ro);
                _ttpms_data.fl_ttpms.temp_data[15] = HYTECH_LF_TTPMS_T16_ro_fromS(unpacked_can_msg.LF_TTPMS_T16_ro);

                break;
            }
        }

        // Front right TTPMS
        {
            case RF_TTPMS_1_CANID:
            {
                RF_TTPMS_1_t unpacked_can_msg;
                Unpack_RF_TTPMS_1_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_RF_TTPMS_1_hytech,
                                    VCRCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.fr_ttpms.serial_number = unpacked_can_msg.RF_TTPMS_SN;
                _ttpms_data.fr_ttpms.bat_voltage = unpacked_can_msg.RF_TTPMS_BAT_V;
                _ttpms_data.fr_ttpms.pressure = HYTECH_RF_TTPMS_P_ro_fromS(unpacked_can_msg.RF_TTPMS_P_ro);
                _ttpms_data.fr_ttpms.gauge_pressure = unpacked_can_msg.RF_TTPMS_P_GAUGE;

                break;
            }
            case RF_TTPMS_2_CANID:
            {
                RF_TTPMS_2_t unpacked_can_msg;
                Unpack_RF_TTPMS_2_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_RF_TTPMS_2_hytech,
                                    VCRCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.fr_ttpms.temp_data[0] = HYTECH_RF_TTPMS_T1_ro_fromS(unpacked_can_msg.RF_TTPMS_T1_ro);
                _ttpms_data.fr_ttpms.temp_data[1] = HYTECH_RF_TTPMS_T2_ro_fromS(unpacked_can_msg.RF_TTPMS_T2_ro);
                _ttpms_data.fr_ttpms.temp_data[2] = HYTECH_RF_TTPMS_T3_ro_fromS(unpacked_can_msg.RF_TTPMS_T3_ro);
                _ttpms_data.fr_ttpms.temp_data[3] = HYTECH_RF_TTPMS_T4_ro_fromS(unpacked_can_msg.RF_TTPMS_T4_ro);

                break;
            }
            case RF_TTPMS_3_CANID:
            {
                RF_TTPMS_3_t unpacked_can_msg;
                Unpack_RF_TTPMS_3_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_RF_TTPMS_3_hytech,
                                    VCRCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.fr_ttpms.temp_data[4] = HYTECH_RF_TTPMS_T5_ro_fromS(unpacked_can_msg.RF_TTPMS_T5_ro);
                _ttpms_data.fr_ttpms.temp_data[5] = HYTECH_RF_TTPMS_T6_ro_fromS(unpacked_can_msg.RF_TTPMS_T6_ro);
                _ttpms_data.fr_ttpms.temp_data[6] = HYTECH_RF_TTPMS_T7_ro_fromS(unpacked_can_msg.RF_TTPMS_T7_ro);
                _ttpms_data.fr_ttpms.temp_data[7] = HYTECH_RF_TTPMS_T8_ro_fromS(unpacked_can_msg.RF_TTPMS_T8_ro);

                break;
            }
            case RF_TTPMS_4_CANID:
            {
                RF_TTPMS_4_t unpacked_can_msg;
                Unpack_RF_TTPMS_4_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_RF_TTPMS_4_hytech,
                                    VCRCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.fr_ttpms.temp_data[8] = HYTECH_RF_TTPMS_T9_ro_fromS(unpacked_can_msg.RF_TTPMS_T9_ro);
                _ttpms_data.fr_ttpms.temp_data[9] = HYTECH_RF_TTPMS_T10_ro_fromS(unpacked_can_msg.RF_TTPMS_T10_ro);
                _ttpms_data.fr_ttpms.temp_data[10] = HYTECH_RF_TTPMS_T11_ro_fromS(unpacked_can_msg.RF_TTPMS_T11_ro);
                _ttpms_data.fr_ttpms.temp_data[11] = HYTECH_RF_TTPMS_T12_ro_fromS(unpacked_can_msg.RF_TTPMS_T12_ro);

                break;
            }
            case RF_TTPMS_5_CANID:
            {
                RF_TTPMS_5_t unpacked_can_msg;
                Unpack_RF_TTPMS_5_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_RF_TTPMS_5_hytech,
                                    VCRCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.fr_ttpms.temp_data[12] = HYTECH_RF_TTPMS_T13_ro_fromS(unpacked_can_msg.RF_TTPMS_T13_ro);
                _ttpms_data.fr_ttpms.temp_data[13] = HYTECH_RF_TTPMS_T14_ro_fromS(unpacked_can_msg.RF_TTPMS_T14_ro);
                _ttpms_data.fr_ttpms.temp_data[14] = HYTECH_RF_TTPMS_T15_ro_fromS(unpacked_can_msg.RF_TTPMS_T15_ro);
                _ttpms_data.fr_ttpms.temp_data[15] = HYTECH_RF_TTPMS_T16_ro_fromS(unpacked_can_msg.RF_TTPMS_T16_ro);

                break;
            }
        }

        // Rear left TTPMS
        {
            case LR_TTPMS_1_CANID:
            {
                LR_TTPMS_1_t unpacked_can_msg;
                Unpack_LR_TTPMS_1_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_LR_TTPMS_1_hytech,
                                    VCRCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.rl_ttpms.serial_number = unpacked_can_msg.LR_TTPMS_SN;
                _ttpms_data.rl_ttpms.bat_voltage = unpacked_can_msg.LR_TTPMS_BAT_V;
                _ttpms_data.rl_ttpms.pressure = HYTECH_LR_TTPMS_P_ro_fromS(unpacked_can_msg.LR_TTPMS_P_ro);
                _ttpms_data.rl_ttpms.gauge_pressure = unpacked_can_msg.LR_TTPMS_P_GAUGE;

                break;
            }
            case LR_TTPMS_2_CANID:
            {
                LR_TTPMS_2_t unpacked_can_msg;
                Unpack_LR_TTPMS_2_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_LR_TTPMS_2_hytech,
                                    VCRCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.rl_ttpms.temp_data[0] = HYTECH_LR_TTPMS_T1_ro_fromS(unpacked_can_msg.LR_TTPMS_T1_ro);
                _ttpms_data.rl_ttpms.temp_data[1] = HYTECH_LR_TTPMS_T2_ro_fromS(unpacked_can_msg.LR_TTPMS_T2_ro);
                _ttpms_data.rl_ttpms.temp_data[2] = HYTECH_LR_TTPMS_T3_ro_fromS(unpacked_can_msg.LR_TTPMS_T3_ro);
                _ttpms_data.rl_ttpms.temp_data[3] = HYTECH_LR_TTPMS_T4_ro_fromS(unpacked_can_msg.LR_TTPMS_T4_ro);

                break;
            }
            case LR_TTPMS_3_CANID:
            {
                LR_TTPMS_3_t unpacked_can_msg;
                Unpack_LR_TTPMS_3_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_LR_TTPMS_3_hytech,
                                    VCRCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.rl_ttpms.temp_data[4] = HYTECH_LR_TTPMS_T5_ro_fromS(unpacked_can_msg.LR_TTPMS_T5_ro);
                _ttpms_data.rl_ttpms.temp_data[5] = HYTECH_LR_TTPMS_T6_ro_fromS(unpacked_can_msg.LR_TTPMS_T6_ro);
                _ttpms_data.rl_ttpms.temp_data[6] = HYTECH_LR_TTPMS_T7_ro_fromS(unpacked_can_msg.LR_TTPMS_T7_ro);
                _ttpms_data.rl_ttpms.temp_data[7] = HYTECH_LR_TTPMS_T8_ro_fromS(unpacked_can_msg.LR_TTPMS_T8_ro);

                break;
            }
            case LR_TTPMS_4_CANID:
            {
                LR_TTPMS_4_t unpacked_can_msg;
                Unpack_LR_TTPMS_4_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_LR_TTPMS_4_hytech,
                                    VCRCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.rl_ttpms.temp_data[8] = HYTECH_LR_TTPMS_T9_ro_fromS(unpacked_can_msg.LR_TTPMS_T9_ro);
                _ttpms_data.rl_ttpms.temp_data[9] = HYTECH_LR_TTPMS_T10_ro_fromS(unpacked_can_msg.LR_TTPMS_T10_ro);
                _ttpms_data.rl_ttpms.temp_data[10] = HYTECH_LR_TTPMS_T11_ro_fromS(unpacked_can_msg.LR_TTPMS_T11_ro);
                _ttpms_data.rl_ttpms.temp_data[11] = HYTECH_LR_TTPMS_T12_ro_fromS(unpacked_can_msg.LR_TTPMS_T12_ro);

                break;
            }
            case LR_TTPMS_5_CANID:
            {
                LR_TTPMS_5_t unpacked_can_msg;
                Unpack_LR_TTPMS_5_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_LR_TTPMS_5_hytech,
                                    VCRCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.rl_ttpms.temp_data[12] = HYTECH_LR_TTPMS_T13_ro_fromS(unpacked_can_msg.LR_TTPMS_T13_ro);
                _ttpms_data.rl_ttpms.temp_data[13] = HYTECH_LR_TTPMS_T14_ro_fromS(unpacked_can_msg.LR_TTPMS_T14_ro);
                _ttpms_data.rl_ttpms.temp_data[14] = HYTECH_LR_TTPMS_T15_ro_fromS(unpacked_can_msg.LR_TTPMS_T15_ro);
                _ttpms_data.rl_ttpms.temp_data[15] = HYTECH_LR_TTPMS_T16_ro_fromS(unpacked_can_msg.LR_TTPMS_T16_ro);

                break;
            }
        }

        // Rear right TTPMS
        {
            case RR_TTPMS_1_CANID:
            {
                RR_TTPMS_1_t unpacked_can_msg;
                Unpack_RR_TTPMS_1_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_RR_TTPMS_1_hytech,
                                    VCRCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.rr_ttpms.serial_number = unpacked_can_msg.RR_TTPMS_SN;
                _ttpms_data.rr_ttpms.bat_voltage = unpacked_can_msg.RR_TTPMS_BAT_V;
                _ttpms_data.rr_ttpms.pressure = HYTECH_RR_TTPMS_P_ro_fromS(unpacked_can_msg.RR_TTPMS_P_ro);
                _ttpms_data.rr_ttpms.gauge_pressure = unpacked_can_msg.RR_TTPMS_P_GAUGE;

                break;
            }
            case RR_TTPMS_2_CANID:
            {
                RR_TTPMS_2_t unpacked_can_msg;
                Unpack_RR_TTPMS_2_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_RR_TTPMS_2_hytech,
                                    VCRCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.rr_ttpms.temp_data[0] = HYTECH_RR_TTPMS_T1_ro_fromS(unpacked_can_msg.RR_TTPMS_T1_ro);
                _ttpms_data.rr_ttpms.temp_data[1] = HYTECH_RR_TTPMS_T2_ro_fromS(unpacked_can_msg.RR_TTPMS_T2_ro);
                _ttpms_data.rr_ttpms.temp_data[2] = HYTECH_RR_TTPMS_T3_ro_fromS(unpacked_can_msg.RR_TTPMS_T3_ro);
                _ttpms_data.rr_ttpms.temp_data[3] = HYTECH_RR_TTPMS_T4_ro_fromS(unpacked_can_msg.RR_TTPMS_T4_ro);

                break;
            }
            case RR_TTPMS_3_CANID:
            {
                RR_TTPMS_3_t unpacked_can_msg;
                Unpack_RR_TTPMS_3_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_RR_TTPMS_3_hytech,
                                    VCRCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.rr_ttpms.temp_data[4] = HYTECH_RR_TTPMS_T5_ro_fromS(unpacked_can_msg.RR_TTPMS_T5_ro);
                _ttpms_data.rr_ttpms.temp_data[5] = HYTECH_RR_TTPMS_T6_ro_fromS(unpacked_can_msg.RR_TTPMS_T6_ro);
                _ttpms_data.rr_ttpms.temp_data[6] = HYTECH_RR_TTPMS_T7_ro_fromS(unpacked_can_msg.RR_TTPMS_T7_ro);
                _ttpms_data.rr_ttpms.temp_data[7] = HYTECH_RR_TTPMS_T8_ro_fromS(unpacked_can_msg.RR_TTPMS_T8_ro);

                break;
            }
            case RR_TTPMS_4_CANID:
            {
                RR_TTPMS_4_t unpacked_can_msg;
                Unpack_RR_TTPMS_4_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_RR_TTPMS_4_hytech,
                                    VCRCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.rr_ttpms.temp_data[8] = HYTECH_RR_TTPMS_T9_ro_fromS(unpacked_can_msg.RR_TTPMS_T9_ro);
                _ttpms_data.rr_ttpms.temp_data[9] = HYTECH_RR_TTPMS_T10_ro_fromS(unpacked_can_msg.RR_TTPMS_T10_ro);
                _ttpms_data.rr_ttpms.temp_data[10] = HYTECH_RR_TTPMS_T11_ro_fromS(unpacked_can_msg.RR_TTPMS_T11_ro);
                _ttpms_data.rr_ttpms.temp_data[11] = HYTECH_RR_TTPMS_T12_ro_fromS(unpacked_can_msg.RR_TTPMS_T12_ro);

                break;
            }
            case RR_TTPMS_5_CANID:
            {
                RR_TTPMS_5_t unpacked_can_msg;
                Unpack_RR_TTPMS_5_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_RR_TTPMS_5_hytech,
                                    VCRCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.rr_ttpms.temp_data[12] = HYTECH_RR_TTPMS_T13_ro_fromS(unpacked_can_msg.RR_TTPMS_T13_ro);
                _ttpms_data.rr_ttpms.temp_data[13] = HYTECH_RR_TTPMS_T14_ro_fromS(unpacked_can_msg.RR_TTPMS_T14_ro);
                _ttpms_data.rr_ttpms.temp_data[14] = HYTECH_RR_TTPMS_T15_ro_fromS(unpacked_can_msg.RR_TTPMS_T15_ro);
                _ttpms_data.rr_ttpms.temp_data[15] = HYTECH_RR_TTPMS_T16_ro_fromS(unpacked_can_msg.RR_TTPMS_T16_ro);

                break;
            }
        }
    }
}

void TTPMSInterface::printTTPMSData(const char* wheel_label, const TTPMSSingleSensorData_s &sensor)
{
    Serial.print("[");
    Serial.print(wheel_label);
    Serial.print("] SN: ");
    Serial.print(sensor.serial_number);
    Serial.print("  BatV: ");
    Serial.print(sensor.bat_voltage);
    Serial.print("  Pressure: ");
    Serial.print(sensor.pressure, 2);
    Serial.print("  GaugeP: ");
    Serial.println(sensor.gauge_pressure);

    Serial.print("[");
    Serial.print(wheel_label);
    Serial.print("] Temps: ");
    for (int i = 0; i < 16; i++)
    {
        Serial.print("T");
        Serial.print(i + 1);
        Serial.print("=");
        Serial.print(sensor.temp_data[i], 2);
        Serial.print(" ");
    }
    Serial.println();
}
#include "TTPMSInterface.h"
#include "VCFCANInterfaceImpl.h"


TTPMSAllSensorData_s TTPMSInterface::getTTPMSData() const
{
    return _ttpms_data;
}

void TTPMSInterface::receiveTTPMSData(const CAN_message_t &can_msg, unsigned long curr_millis)
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
                                    VCFCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.fl_ttpms.serial_number = unpacked_can_msg.LF_TTPMS_SN;
                _ttpms_data.fl_ttpms.battery_voltage_mv = unpacked_can_msg.LF_TTPMS_BAT_V;
                _ttpms_data.fl_ttpms.pressure = _hsPressureFromRaw(unpacked_can_msg.LF_TTPMS_P_ro);
                _ttpms_data.fl_ttpms.gauge_pressure = unpacked_can_msg.LF_TTPMS_P_GAUGE;
                _ttpms_data.fl_ttpms.last_pressure_recv_millis = curr_millis;
                _updateHealth(_ttpms_data.fl_ttpms);

                break;
            }
            case LF_TTPMS_2_CANID:
            {
                LF_TTPMS_2_t unpacked_can_msg;
                Unpack_LF_TTPMS_2_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_LF_TTPMS_2_hytech,
                                    VCFCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.fl_ttpms.temp_data[0] = HYTECH_LF_TTPMS_T1_ro_fromS(unpacked_can_msg.LF_TTPMS_T1_ro);
                _ttpms_data.fl_ttpms.temp_data[1] = HYTECH_LF_TTPMS_T2_ro_fromS(unpacked_can_msg.LF_TTPMS_T2_ro);
                _ttpms_data.fl_ttpms.temp_data[2] = HYTECH_LF_TTPMS_T3_ro_fromS(unpacked_can_msg.LF_TTPMS_T3_ro);
                _ttpms_data.fl_ttpms.temp_data[3] = HYTECH_LF_TTPMS_T4_ro_fromS(unpacked_can_msg.LF_TTPMS_T4_ro);
                _ttpms_data.fl_ttpms.last_temp_recv_millis = curr_millis;

                break;
            }
            case LF_TTPMS_3_CANID:
            {
                LF_TTPMS_3_t unpacked_can_msg;
                Unpack_LF_TTPMS_3_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_LF_TTPMS_3_hytech,
                                    VCFCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.fl_ttpms.temp_data[4] = HYTECH_LF_TTPMS_T5_ro_fromS(unpacked_can_msg.LF_TTPMS_T5_ro);
                _ttpms_data.fl_ttpms.temp_data[5] = HYTECH_LF_TTPMS_T6_ro_fromS(unpacked_can_msg.LF_TTPMS_T6_ro);
                _ttpms_data.fl_ttpms.temp_data[6] = HYTECH_LF_TTPMS_T7_ro_fromS(unpacked_can_msg.LF_TTPMS_T7_ro);
                _ttpms_data.fl_ttpms.temp_data[7] = HYTECH_LF_TTPMS_T8_ro_fromS(unpacked_can_msg.LF_TTPMS_T8_ro);
                _ttpms_data.fl_ttpms.last_temp_recv_millis = curr_millis;
                break;
            }
            case LF_TTPMS_4_CANID:
            {
                LF_TTPMS_4_t unpacked_can_msg;
                Unpack_LF_TTPMS_4_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_LF_TTPMS_4_hytech,
                                    VCFCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.fl_ttpms.temp_data[8] = HYTECH_LF_TTPMS_T9_ro_fromS(unpacked_can_msg.LF_TTPMS_T9_ro);
                _ttpms_data.fl_ttpms.temp_data[9] = HYTECH_LF_TTPMS_T10_ro_fromS(unpacked_can_msg.LF_TTPMS_T10_ro);
                _ttpms_data.fl_ttpms.temp_data[10] = HYTECH_LF_TTPMS_T11_ro_fromS(unpacked_can_msg.LF_TTPMS_T11_ro);
                _ttpms_data.fl_ttpms.temp_data[11] = HYTECH_LF_TTPMS_T12_ro_fromS(unpacked_can_msg.LF_TTPMS_T12_ro);
                _ttpms_data.fl_ttpms.last_temp_recv_millis = curr_millis;

                break;
            }
            case LF_TTPMS_5_CANID:
            {
                LF_TTPMS_5_t unpacked_can_msg;
                Unpack_LF_TTPMS_5_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_LF_TTPMS_5_hytech,
                                    VCFCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.fl_ttpms.temp_data[12] = HYTECH_LF_TTPMS_T13_ro_fromS(unpacked_can_msg.LF_TTPMS_T13_ro);
                _ttpms_data.fl_ttpms.temp_data[13] = HYTECH_LF_TTPMS_T14_ro_fromS(unpacked_can_msg.LF_TTPMS_T14_ro);
                _ttpms_data.fl_ttpms.temp_data[14] = HYTECH_LF_TTPMS_T15_ro_fromS(unpacked_can_msg.LF_TTPMS_T15_ro);
                _ttpms_data.fl_ttpms.temp_data[15] = HYTECH_LF_TTPMS_T16_ro_fromS(unpacked_can_msg.LF_TTPMS_T16_ro);
                _ttpms_data.fl_ttpms.last_temp_recv_millis = curr_millis;

                break;
            }
            case LF_TTPMS_6_CANID:
            {
                LF_TTPMS_6_t unpacked_can_msg;
                Unpack_LF_TTPMS_6_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_LF_TTPMS_6_hytech,
                                    VCFCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.fl_ttpms.transmission_count = unpacked_can_msg.LF_TTPMS_TC;
                _ttpms_data.fl_ttpms.rssi_dbm = unpacked_can_msg.LF_TTPMS_RSSI;
                _ttpms_data.fl_ttpms.sensor_temp = HYTECH_LF_TTPMS_T_ro_fromS(unpacked_can_msg.LF_TTPMS_T_ro);
                _ttpms_data.fl_ttpms.node_id = unpacked_can_msg.LF_TTPMS_NODE_ID;
                _ttpms_data.fl_ttpms.last_status_recv_millis = curr_millis;
                _updateHealth(_ttpms_data.fl_ttpms);

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
                                    VCFCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.fr_ttpms.serial_number = unpacked_can_msg.RF_TTPMS_SN;
                _ttpms_data.fr_ttpms.battery_voltage_mv = unpacked_can_msg.RF_TTPMS_BAT_V;
                _ttpms_data.fr_ttpms.pressure = _hsPressureFromRaw(unpacked_can_msg.RF_TTPMS_P_ro);
                _ttpms_data.fr_ttpms.gauge_pressure = unpacked_can_msg.RF_TTPMS_P_GAUGE;
                _ttpms_data.fr_ttpms.last_pressure_recv_millis = curr_millis;
                _updateHealth(_ttpms_data.fr_ttpms);

                break;
            }
            case RF_TTPMS_2_CANID:
            {
                RF_TTPMS_2_t unpacked_can_msg;
                Unpack_RF_TTPMS_2_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_RF_TTPMS_2_hytech,
                                    VCFCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.fr_ttpms.temp_data[0] = HYTECH_RF_TTPMS_T1_ro_fromS(unpacked_can_msg.RF_TTPMS_T1_ro);
                _ttpms_data.fr_ttpms.temp_data[1] = HYTECH_RF_TTPMS_T2_ro_fromS(unpacked_can_msg.RF_TTPMS_T2_ro);
                _ttpms_data.fr_ttpms.temp_data[2] = HYTECH_RF_TTPMS_T3_ro_fromS(unpacked_can_msg.RF_TTPMS_T3_ro);
                _ttpms_data.fr_ttpms.temp_data[3] = HYTECH_RF_TTPMS_T4_ro_fromS(unpacked_can_msg.RF_TTPMS_T4_ro);
                _ttpms_data.fr_ttpms.last_temp_recv_millis = curr_millis;

                break;
            }
            case RF_TTPMS_3_CANID:
            {
                RF_TTPMS_3_t unpacked_can_msg;
                Unpack_RF_TTPMS_3_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_RF_TTPMS_3_hytech,
                                    VCFCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.fr_ttpms.temp_data[4] = HYTECH_RF_TTPMS_T5_ro_fromS(unpacked_can_msg.RF_TTPMS_T5_ro);
                _ttpms_data.fr_ttpms.temp_data[5] = HYTECH_RF_TTPMS_T6_ro_fromS(unpacked_can_msg.RF_TTPMS_T6_ro);
                _ttpms_data.fr_ttpms.temp_data[6] = HYTECH_RF_TTPMS_T7_ro_fromS(unpacked_can_msg.RF_TTPMS_T7_ro);
                _ttpms_data.fr_ttpms.temp_data[7] = HYTECH_RF_TTPMS_T8_ro_fromS(unpacked_can_msg.RF_TTPMS_T8_ro);
                _ttpms_data.fr_ttpms.last_temp_recv_millis = curr_millis;

                break;
            }
            case RF_TTPMS_4_CANID:
            {
                RF_TTPMS_4_t unpacked_can_msg;
                Unpack_RF_TTPMS_4_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_RF_TTPMS_4_hytech,
                                    VCFCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.fr_ttpms.temp_data[8] = HYTECH_RF_TTPMS_T9_ro_fromS(unpacked_can_msg.RF_TTPMS_T9_ro);
                _ttpms_data.fr_ttpms.temp_data[9] = HYTECH_RF_TTPMS_T10_ro_fromS(unpacked_can_msg.RF_TTPMS_T10_ro);
                _ttpms_data.fr_ttpms.temp_data[10] = HYTECH_RF_TTPMS_T11_ro_fromS(unpacked_can_msg.RF_TTPMS_T11_ro);
                _ttpms_data.fr_ttpms.temp_data[11] = HYTECH_RF_TTPMS_T12_ro_fromS(unpacked_can_msg.RF_TTPMS_T12_ro);
                _ttpms_data.fr_ttpms.last_temp_recv_millis = curr_millis;

                break;
            }
            case RF_TTPMS_5_CANID:
            {
                RF_TTPMS_5_t unpacked_can_msg;
                Unpack_RF_TTPMS_5_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_RF_TTPMS_5_hytech,
                                    VCFCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.fr_ttpms.temp_data[12] = HYTECH_RF_TTPMS_T13_ro_fromS(unpacked_can_msg.RF_TTPMS_T13_ro);
                _ttpms_data.fr_ttpms.temp_data[13] = HYTECH_RF_TTPMS_T14_ro_fromS(unpacked_can_msg.RF_TTPMS_T14_ro);
                _ttpms_data.fr_ttpms.temp_data[14] = HYTECH_RF_TTPMS_T15_ro_fromS(unpacked_can_msg.RF_TTPMS_T15_ro);
                _ttpms_data.fr_ttpms.temp_data[15] = HYTECH_RF_TTPMS_T16_ro_fromS(unpacked_can_msg.RF_TTPMS_T16_ro);
                _ttpms_data.fr_ttpms.last_temp_recv_millis = curr_millis;

                break;
            }
            case RF_TTPMS_6_CANID:
            {
                RF_TTPMS_6_t unpacked_can_msg;
                Unpack_RF_TTPMS_6_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_RF_TTPMS_6_hytech,
                                    VCFCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.fr_ttpms.transmission_count = unpacked_can_msg.RF_TTPMS_TC;
                _ttpms_data.fr_ttpms.rssi_dbm = unpacked_can_msg.RF_TTPMS_RSSI;
                _ttpms_data.fr_ttpms.sensor_temp = HYTECH_RF_TTPMS_T_ro_fromS(unpacked_can_msg.RF_TTPMS_T_ro);
                _ttpms_data.fr_ttpms.node_id = unpacked_can_msg.RF_TTPMS_NODE_ID;
                _ttpms_data.fr_ttpms.last_status_recv_millis = curr_millis;
                _updateHealth(_ttpms_data.fr_ttpms);

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
                                    VCFCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.rl_ttpms.serial_number = unpacked_can_msg.LR_TTPMS_SN;
                _ttpms_data.rl_ttpms.battery_voltage_mv = unpacked_can_msg.LR_TTPMS_BAT_V;
                _ttpms_data.rl_ttpms.pressure = _hsPressureFromRaw(unpacked_can_msg.LR_TTPMS_P_ro);
                _ttpms_data.rl_ttpms.gauge_pressure = unpacked_can_msg.LR_TTPMS_P_GAUGE;
                _ttpms_data.rl_ttpms.last_pressure_recv_millis = curr_millis;
                _updateHealth(_ttpms_data.rl_ttpms);

                break;
            }
            case LR_TTPMS_2_CANID:
            {
                LR_TTPMS_2_t unpacked_can_msg;
                Unpack_LR_TTPMS_2_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_LR_TTPMS_2_hytech,
                                    VCFCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.rl_ttpms.temp_data[0] = HYTECH_LR_TTPMS_T1_ro_fromS(unpacked_can_msg.LR_TTPMS_T1_ro);
                _ttpms_data.rl_ttpms.temp_data[1] = HYTECH_LR_TTPMS_T2_ro_fromS(unpacked_can_msg.LR_TTPMS_T2_ro);
                _ttpms_data.rl_ttpms.temp_data[2] = HYTECH_LR_TTPMS_T3_ro_fromS(unpacked_can_msg.LR_TTPMS_T3_ro);
                _ttpms_data.rl_ttpms.temp_data[3] = HYTECH_LR_TTPMS_T4_ro_fromS(unpacked_can_msg.LR_TTPMS_T4_ro);
                _ttpms_data.rl_ttpms.last_temp_recv_millis = curr_millis;

                break;
            }
            case LR_TTPMS_3_CANID:
            {
                LR_TTPMS_3_t unpacked_can_msg;
                Unpack_LR_TTPMS_3_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_LR_TTPMS_3_hytech,
                                    VCFCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.rl_ttpms.temp_data[4] = HYTECH_LR_TTPMS_T5_ro_fromS(unpacked_can_msg.LR_TTPMS_T5_ro);
                _ttpms_data.rl_ttpms.temp_data[5] = HYTECH_LR_TTPMS_T6_ro_fromS(unpacked_can_msg.LR_TTPMS_T6_ro);
                _ttpms_data.rl_ttpms.temp_data[6] = HYTECH_LR_TTPMS_T7_ro_fromS(unpacked_can_msg.LR_TTPMS_T7_ro);
                _ttpms_data.rl_ttpms.temp_data[7] = HYTECH_LR_TTPMS_T8_ro_fromS(unpacked_can_msg.LR_TTPMS_T8_ro);
                _ttpms_data.rl_ttpms.last_temp_recv_millis = curr_millis;

                break;
            }
            case LR_TTPMS_4_CANID:
            {
                LR_TTPMS_4_t unpacked_can_msg;
                Unpack_LR_TTPMS_4_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_LR_TTPMS_4_hytech,
                                    VCFCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.rl_ttpms.temp_data[8] = HYTECH_LR_TTPMS_T9_ro_fromS(unpacked_can_msg.LR_TTPMS_T9_ro);
                _ttpms_data.rl_ttpms.temp_data[9] = HYTECH_LR_TTPMS_T10_ro_fromS(unpacked_can_msg.LR_TTPMS_T10_ro);
                _ttpms_data.rl_ttpms.temp_data[10] = HYTECH_LR_TTPMS_T11_ro_fromS(unpacked_can_msg.LR_TTPMS_T11_ro);
                _ttpms_data.rl_ttpms.temp_data[11] = HYTECH_LR_TTPMS_T12_ro_fromS(unpacked_can_msg.LR_TTPMS_T12_ro);
                _ttpms_data.rl_ttpms.last_temp_recv_millis = curr_millis;

                break;
            }
            case LR_TTPMS_5_CANID:
            {
                LR_TTPMS_5_t unpacked_can_msg;
                Unpack_LR_TTPMS_5_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_LR_TTPMS_5_hytech,
                                    VCFCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.rl_ttpms.temp_data[12] = HYTECH_LR_TTPMS_T13_ro_fromS(unpacked_can_msg.LR_TTPMS_T13_ro);
                _ttpms_data.rl_ttpms.temp_data[13] = HYTECH_LR_TTPMS_T14_ro_fromS(unpacked_can_msg.LR_TTPMS_T14_ro);
                _ttpms_data.rl_ttpms.temp_data[14] = HYTECH_LR_TTPMS_T15_ro_fromS(unpacked_can_msg.LR_TTPMS_T15_ro);
                _ttpms_data.rl_ttpms.temp_data[15] = HYTECH_LR_TTPMS_T16_ro_fromS(unpacked_can_msg.LR_TTPMS_T16_ro);
                _ttpms_data.rl_ttpms.last_temp_recv_millis = curr_millis;

                break;
            }
            case LR_TTPMS_6_CANID:
            {
                LR_TTPMS_6_t unpacked_can_msg;
                Unpack_LR_TTPMS_6_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_LR_TTPMS_6_hytech,
                                    VCFCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.rl_ttpms.transmission_count = unpacked_can_msg.LR_TTPMS_TC;
                _ttpms_data.rl_ttpms.rssi_dbm = unpacked_can_msg.LR_TTPMS_RSSI;
                _ttpms_data.rl_ttpms.sensor_temp = HYTECH_LR_TTPMS_T_ro_fromS(unpacked_can_msg.LR_TTPMS_T_ro);
                _ttpms_data.rl_ttpms.node_id = unpacked_can_msg.LR_TTPMS_NODE_ID;
                _ttpms_data.rl_ttpms.last_status_recv_millis = curr_millis;
                _updateHealth(_ttpms_data.rl_ttpms);

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
                                    VCFCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.rr_ttpms.serial_number = unpacked_can_msg.RR_TTPMS_SN;
                _ttpms_data.rr_ttpms.battery_voltage_mv = unpacked_can_msg.RR_TTPMS_BAT_V;
                _ttpms_data.rr_ttpms.pressure = _hsPressureFromRaw(unpacked_can_msg.RR_TTPMS_P_ro);
                _ttpms_data.rr_ttpms.gauge_pressure = unpacked_can_msg.RR_TTPMS_P_GAUGE;
                _ttpms_data.rr_ttpms.last_pressure_recv_millis = curr_millis;
                _updateHealth(_ttpms_data.rr_ttpms);

                break;
            }
            case RR_TTPMS_2_CANID:
            {
                RR_TTPMS_2_t unpacked_can_msg;
                Unpack_RR_TTPMS_2_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_RR_TTPMS_2_hytech,
                                    VCFCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.rr_ttpms.temp_data[0] = HYTECH_RR_TTPMS_T1_ro_fromS(unpacked_can_msg.RR_TTPMS_T1_ro);
                _ttpms_data.rr_ttpms.temp_data[1] = HYTECH_RR_TTPMS_T2_ro_fromS(unpacked_can_msg.RR_TTPMS_T2_ro);
                _ttpms_data.rr_ttpms.temp_data[2] = HYTECH_RR_TTPMS_T3_ro_fromS(unpacked_can_msg.RR_TTPMS_T3_ro);
                _ttpms_data.rr_ttpms.temp_data[3] = HYTECH_RR_TTPMS_T4_ro_fromS(unpacked_can_msg.RR_TTPMS_T4_ro);
                _ttpms_data.rr_ttpms.last_temp_recv_millis = curr_millis;

                break;
            }
            case RR_TTPMS_3_CANID:
            {
                RR_TTPMS_3_t unpacked_can_msg;
                Unpack_RR_TTPMS_3_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_RR_TTPMS_3_hytech,
                                    VCFCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.rr_ttpms.temp_data[4] = HYTECH_RR_TTPMS_T5_ro_fromS(unpacked_can_msg.RR_TTPMS_T5_ro);
                _ttpms_data.rr_ttpms.temp_data[5] = HYTECH_RR_TTPMS_T6_ro_fromS(unpacked_can_msg.RR_TTPMS_T6_ro);
                _ttpms_data.rr_ttpms.temp_data[6] = HYTECH_RR_TTPMS_T7_ro_fromS(unpacked_can_msg.RR_TTPMS_T7_ro);
                _ttpms_data.rr_ttpms.temp_data[7] = HYTECH_RR_TTPMS_T8_ro_fromS(unpacked_can_msg.RR_TTPMS_T8_ro);
                _ttpms_data.rr_ttpms.last_temp_recv_millis = curr_millis;

                break;
            }
            case RR_TTPMS_4_CANID:
            {
                RR_TTPMS_4_t unpacked_can_msg;
                Unpack_RR_TTPMS_4_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_RR_TTPMS_4_hytech,
                                    VCFCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.rr_ttpms.temp_data[8] = HYTECH_RR_TTPMS_T9_ro_fromS(unpacked_can_msg.RR_TTPMS_T9_ro);
                _ttpms_data.rr_ttpms.temp_data[9] = HYTECH_RR_TTPMS_T10_ro_fromS(unpacked_can_msg.RR_TTPMS_T10_ro);
                _ttpms_data.rr_ttpms.temp_data[10] = HYTECH_RR_TTPMS_T11_ro_fromS(unpacked_can_msg.RR_TTPMS_T11_ro);
                _ttpms_data.rr_ttpms.temp_data[11] = HYTECH_RR_TTPMS_T12_ro_fromS(unpacked_can_msg.RR_TTPMS_T12_ro);
                _ttpms_data.rr_ttpms.last_temp_recv_millis = curr_millis;

                break;
            }
            case RR_TTPMS_5_CANID:
            {
                RR_TTPMS_5_t unpacked_can_msg;
                Unpack_RR_TTPMS_5_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_RR_TTPMS_5_hytech,
                                    VCFCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.rr_ttpms.temp_data[12] = HYTECH_RR_TTPMS_T13_ro_fromS(unpacked_can_msg.RR_TTPMS_T13_ro);
                _ttpms_data.rr_ttpms.temp_data[13] = HYTECH_RR_TTPMS_T14_ro_fromS(unpacked_can_msg.RR_TTPMS_T14_ro);
                _ttpms_data.rr_ttpms.temp_data[14] = HYTECH_RR_TTPMS_T15_ro_fromS(unpacked_can_msg.RR_TTPMS_T15_ro);
                _ttpms_data.rr_ttpms.temp_data[15] = HYTECH_RR_TTPMS_T16_ro_fromS(unpacked_can_msg.RR_TTPMS_T16_ro);
                _ttpms_data.rr_ttpms.last_temp_recv_millis = curr_millis;

                break;
            }
            case RR_TTPMS_6_CANID:
            {
                RR_TTPMS_6_t unpacked_can_msg;
                Unpack_RR_TTPMS_6_hytech(&unpacked_can_msg, can_msg.buf, can_msg.len);
                CAN_util::enqueue_msg(&unpacked_can_msg,
                                    &Pack_RR_TTPMS_6_hytech,
                                    VCFCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id
                );

                _ttpms_data.rr_ttpms.transmission_count = unpacked_can_msg.RR_TTPMS_TC;
                _ttpms_data.rr_ttpms.rssi_dbm = unpacked_can_msg.RR_TTPMS_RSSI;
                _ttpms_data.rr_ttpms.sensor_temp = HYTECH_RR_TTPMS_T_ro_fromS(unpacked_can_msg.RR_TTPMS_T_ro);
                _ttpms_data.rr_ttpms.node_id = unpacked_can_msg.RR_TTPMS_NODE_ID;
                _ttpms_data.rr_ttpms.last_status_recv_millis = curr_millis;
                _updateHealth(_ttpms_data.rr_ttpms);

                break;
            }
        }
    }
}

bool TTPMSInterface::isSensorActive(const TTPMSSingleSensorData_s &sensor, unsigned long curr_millis, unsigned long timeout_ms)
{
    return (sensor.last_pressure_recv_millis != 0) && ((curr_millis - sensor.last_pressure_recv_millis) <= timeout_ms);
}

bool TTPMSInterface::buildBaseConfigMsg(TTPMSCorner_e corner, uint16_t new_base_id, TTPMSBitRate_e bit_rate,
                                        TTPMSEmissivityMode_e emissivity_mode, CAN_message_t &msg_out) const
{
    uint8_t bit_rate_raw = static_cast<uint8_t>(bit_rate);
    uint8_t emissivity_raw = static_cast<uint8_t>(emissivity_mode);
    if (new_base_id < _params.min_base_can_id || new_base_id > _params.max_base_can_id ||
        bit_rate_raw < static_cast<uint8_t>(TTPMSBitRate_e::BITRATE_1MBPS) ||
        bit_rate_raw > static_cast<uint8_t>(TTPMSBitRate_e::BITRATE_100KBPS) ||
        emissivity_raw > static_cast<uint8_t>(TTPMSEmissivityMode_e::CUSTOM))
    {
        return false;
    }

    _packCommandID(msg_out, corner, _params.command_ids.base_config);
    msg_out.buf[2] = static_cast<uint8_t>(new_base_id >> 8);
    msg_out.buf[3] = static_cast<uint8_t>(new_base_id & 0xFF);
    msg_out.buf[4] = _params.sensor_assignment_default;
    msg_out.buf[5] = bit_rate_raw;
    msg_out.buf[6] = emissivity_raw;
    msg_out.buf[7] = 0;
    return true;
}

bool TTPMSInterface::buildRFConfigMsg(TTPMSCorner_e corner, uint8_t network_id, uint8_t node_id,
                                      uint32_t frequency_hz, CAN_message_t &msg_out) const
{
    if (frequency_hz < _params.min_rf_frequency_hz || frequency_hz > _params.max_rf_frequency_hz ||
        (frequency_hz % _params.rf_frequency_resolution_hz) != 0)
    {
        return false;
    }

    uint16_t frequency_raw = static_cast<uint16_t>(frequency_hz / _params.rf_frequency_resolution_hz);
    _packCommandID(msg_out, corner, _params.command_ids.rf_config);
    msg_out.buf[2] = network_id;
    msg_out.buf[3] = node_id;
    msg_out.buf[4] = static_cast<uint8_t>(frequency_raw >> 8);
    msg_out.buf[5] = static_cast<uint8_t>(frequency_raw & 0xFF);
    msg_out.buf[6] = 0;
    msg_out.buf[7] = 0;
    return true;
}

bool TTPMSInterface::buildEmissivityConfigMsgs(TTPMSCorner_e corner, const std::array<float, TEMP_CHANNELS> &emissivity,
                                               std::array<CAN_message_t, 3> &msgs_out) const
{
    std::array<uint8_t, TEMP_CHANNELS> emissivity_raw;
    for (size_t i = 0; i < TEMP_CHANNELS; i++)
    {
        // Small tolerance so values like 0.01f / 1.0f are not rejected by float rounding
        if (emissivity[i] < (_params.min_emissivity - 0.001f) || emissivity[i] > (_params.max_emissivity + 0.001f))
        {
            return false;
        }
        emissivity_raw[i] = static_cast<uint8_t>((emissivity[i] * 100.0f) + 0.5f); // 0.01 per bit
    }

    // T1 - T6
    _packCommandID(msgs_out[0], corner, _params.command_ids.emissivity_t1_t6);
    for (size_t i = 0; i < 6; i++)
    {
        msgs_out[0].buf[2 + i] = emissivity_raw[i];
    }

    // T7 - T12
    _packCommandID(msgs_out[1], corner, _params.command_ids.emissivity_t7_t12);
    for (size_t i = 0; i < 6; i++)
    {
        msgs_out[1].buf[2 + i] = emissivity_raw[6 + i];
    }

    // T13 - T16, then the default emissivity which must not be changed
    _packCommandID(msgs_out[2], corner, _params.command_ids.emissivity_t13_t16);
    for (size_t i = 0; i < 4; i++)
    {
        msgs_out[2].buf[2 + i] = emissivity_raw[12 + i];
    }
    msgs_out[2].buf[6] = _params.emissivity_raw_default;
    msgs_out[2].buf[7] = 0;
    return true;
}

bool TTPMSInterface::startConfigSequence(const CAN_message_t *msgs, size_t num_msgs, unsigned long curr_millis)
{
    if (_config_active || msgs == nullptr || num_msgs == 0 || num_msgs > ttpms_default_params::MAX_CONFIG_MSGS)
    {
        return false;
    }

    for (size_t i = 0; i < num_msgs; i++)
    {
        _config_msgs[i] = msgs[i];
    }
    _num_config_msgs = num_msgs;
    _config_active = true;
    _config_start_millis = curr_millis;
    _last_config_send_millis = curr_millis - _params.config_send_period_ms; // send on first update
    return true;
}

void TTPMSInterface::updateConfigSequence(unsigned long curr_millis)
{
    if (!_config_active)
    {
        return;
    }

    if ((curr_millis - _config_start_millis) > _params.config_send_duration_ms)
    {
        // Datasheet: do NOT continuously send CAN messages to the receiver. Power cycle it now to apply.
        _config_active = false;
        _num_config_msgs = 0;
        return;
    }

    if ((curr_millis - _last_config_send_millis) >= _params.config_send_period_ms)
    {
        for (size_t i = 0; i < _num_config_msgs; i++)
        {
            uint8_t buf[sizeof(CAN_message_t)] = {};
            memmove(buf, &_config_msgs[i], sizeof(CAN_message_t));
            VCFCANInterfaceInstance::instance().front_aux_can_tx_buffer.push_back(buf, sizeof(CAN_message_t));
        }
        _last_config_send_millis = curr_millis;
    }
}

uint16_t TTPMSInterface::_getReceiverBaseID(TTPMSCorner_e corner) const
{
    switch (corner)
    {
        case TTPMSCorner_e::LF: return LF_TTPMS_1_CANID;
        case TTPMSCorner_e::RF: return RF_TTPMS_1_CANID;
        case TTPMSCorner_e::LR: return LR_TTPMS_1_CANID;
        case TTPMSCorner_e::RR: return RR_TTPMS_1_CANID;
    }
    return LF_TTPMS_1_CANID;
}

void TTPMSInterface::_updateHealth(TTPMSSingleSensorData_s &sensor) const
{
    // Only flag once the relevant message has actually been received
    sensor.is_battery_low = (sensor.last_pressure_recv_millis != 0) && (sensor.battery_voltage_mv < _params.low_battery_threshold_mv);
    sensor.is_rssi_weak = (sensor.last_status_recv_millis != 0) && (sensor.rssi_dbm < _params.min_rssi_dbm);
}

float TTPMSInterface::_hsPressureFromRaw(uint16_t raw) const
{
    // Datasheet specifies HS pressure as signed, but the DBC declares it unsigned, so reinterpret here.
    // Without this, absolute pressures below 3000 mBar (typical FSAE tire pressures) decode as ~9000+ mBar.
    return (static_cast<float>(static_cast<int16_t>(raw)) * _params.hs_pressure_scale_mbar) + _params.hs_pressure_offset_mbar;
}

void TTPMSInterface::_packCommandID(CAN_message_t &msg, TTPMSCorner_e corner, uint16_t command_id) const
{
    msg = CAN_message_t();
    msg.id = _getReceiverBaseID(corner);
    msg.len = 8;
    msg.flags.extended = 0;
    msg.buf[0] = static_cast<uint8_t>(command_id >> 8);  // Big-endian / Motorola
    msg.buf[1] = static_cast<uint8_t>(command_id & 0xFF);
}
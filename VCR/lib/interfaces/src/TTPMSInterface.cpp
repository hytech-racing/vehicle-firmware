#include "TTPMSInterface.h"

TTPMSAllSensorData_s TTPMSInterface::get_ttpms_data() const {
    return _ttpms_data;
}

// also have a function for programming/setting the CAN ID (pg 4 of datasheet) --> send CAN msg?
// alt implementation: have funcs for  5 types of messages sent from each TTPMS

void TTPMSInterface::receive_ttpms_data(const CAN_message_t &msg) {
    switch (msg.id) {
        // Left front TTPMS
        {
            case LF_TTPMS_1_CANID:
            {
                LF_TTPMS_1_t unpacked_msg;
                Unpack_LF_TTPMS_1_hytech(&unpacked_msg, msg.buf, msg.len);
                LF_TTPMS_1_t* p_unpacked_msg = &unpacked_msg;
                _ttpms_data.lf_ttpms.bat_voltage = p_unpacked_msg->LF_TTPMS_BAT_V;
                _ttpms_data.lf_ttpms.pressure = HYTECH_LF_TTPMS_P_ro_fromS(p_unpacked_msg->LF_TTPMS_P_ro); 
                _ttpms_data.lf_ttpms.pressure = p_unpacked_msg->LF_TTPMS_P_GAUGE; // also need conversion func. for gauge pressure?       
                break;
            }
            case LF_TTPMS_2_CANID:
            {
                LF_TTPMS_2_t unpacked_msg;
                Unpack_LF_TTPMS_2_hytech(&unpacked_msg, msg.buf, msg.len);
                LF_TTPMS_2_t* p_unpacked_msg = &unpacked_msg;
                _ttpms_data.lf_ttpms.temp_data[0] = HYTECH_LF_TTPMS_T1_ro_fromS(p_unpacked_msg->LF_TTPMS_T1_ro); // unpack struct pointer OR just use struct attribute?
                _ttpms_data.lf_ttpms.temp_data[1] = HYTECH_LF_TTPMS_T2_ro_fromS(p_unpacked_msg->LF_TTPMS_T2_ro);
                _ttpms_data.lf_ttpms.temp_data[2] = HYTECH_LF_TTPMS_T3_ro_fromS(p_unpacked_msg->LF_TTPMS_T3_ro);
                _ttpms_data.lf_ttpms.temp_data[3] = HYTECH_LF_TTPMS_T4_ro_fromS(p_unpacked_msg->LF_TTPMS_T4_ro);
                break;
            }
            case LF_TTPMS_3_CANID:
            {
                LF_TTPMS_3_t unpacked_msg;
                Unpack_LF_TTPMS_3_hytech(&unpacked_msg, msg.buf, msg.len);
                LF_TTPMS_3_t* p_unpacked_msg = &unpacked_msg;
                _ttpms_data.lf_ttpms.temp_data[4] = HYTECH_LF_TTPMS_T5_ro_fromS(p_unpacked_msg->LF_TTPMS_T5_ro); 
                _ttpms_data.lf_ttpms.temp_data[5] = HYTECH_LF_TTPMS_T6_ro_fromS(p_unpacked_msg->LF_TTPMS_T6_ro);
                _ttpms_data.lf_ttpms.temp_data[6] = HYTECH_LF_TTPMS_T7_ro_fromS(p_unpacked_msg->LF_TTPMS_T7_ro);
                _ttpms_data.lf_ttpms.temp_data[7] = HYTECH_LF_TTPMS_T8_ro_fromS(p_unpacked_msg->LF_TTPMS_T8_ro);
                break;
            }
            case LF_TTPMS_4_CANID:
            {
                LF_TTPMS_4_t unpacked_msg;
                Unpack_LF_TTPMS_4_hytech(&unpacked_msg, msg.buf, msg.len);
                LF_TTPMS_4_t* p_unpacked_msg = &unpacked_msg;
                _ttpms_data.lf_ttpms.temp_data[8] = HYTECH_LF_TTPMS_T9_ro_fromS(p_unpacked_msg->LF_TTPMS_T9_ro); 
                _ttpms_data.lf_ttpms.temp_data[9] = HYTECH_LF_TTPMS_T10_ro_fromS(p_unpacked_msg->LF_TTPMS_T10_ro);
                _ttpms_data.lf_ttpms.temp_data[10] = HYTECH_LF_TTPMS_T11_ro_fromS(p_unpacked_msg->LF_TTPMS_T11_ro);
                _ttpms_data.lf_ttpms.temp_data[11] = HYTECH_LF_TTPMS_T12_ro_fromS(p_unpacked_msg->LF_TTPMS_T12_ro);
                break;
            }
            case LF_TTPMS_5_CANID:
            {
                LF_TTPMS_5_t unpacked_msg;
                Unpack_LF_TTPMS_5_hytech(&unpacked_msg, msg.buf, msg.len);
                LF_TTPMS_5_t* p_unpacked_msg = &unpacked_msg;
                _ttpms_data.lf_ttpms.temp_data[12] = HYTECH_LF_TTPMS_T13_ro_fromS(p_unpacked_msg->LF_TTPMS_T13_ro); 
                _ttpms_data.lf_ttpms.temp_data[13] = HYTECH_LF_TTPMS_T14_ro_fromS(p_unpacked_msg->LF_TTPMS_T14_ro);
                _ttpms_data.lf_ttpms.temp_data[14] = HYTECH_LF_TTPMS_T15_ro_fromS(p_unpacked_msg->LF_TTPMS_T15_ro);
                _ttpms_data.lf_ttpms.temp_data[15] = HYTECH_LF_TTPMS_T16_ro_fromS(p_unpacked_msg->LF_TTPMS_T16_ro);
                break;
            }
        }

        // Front right TTPMS
        {
            case RF_TTPMS_1_CANID:
            {
                RF_TTPMS_1_t unpacked_msg;
                Unpack_RF_TTPMS_1_hytech(&unpacked_msg, msg.buf, msg.len);
                RF_TTPMS_1_t* p_unpacked_msg = &unpacked_msg;
                _ttpms_data.rf_ttpms.bat_voltage = p_unpacked_msg->RF_TTPMS_BAT_V;
                _ttpms_data.rf_ttpms.pressure = p_unpacked_msg->RF_TTPMS_P_ro;
                _ttpms_data.rf_ttpms.pressure = p_unpacked_msg->RF_TTPMS_P_GAUGE;   
                break;
            }
            case RF_TTPMS_2_CANID:
            {
                RF_TTPMS_2_t unpacked_msg;
                Unpack_RF_TTPMS_2_hytech(&unpacked_msg, msg.buf, msg.len);
                RF_TTPMS_2_t* p_unpacked_msg = &unpacked_msg;
                _ttpms_data.rf_ttpms.temp_data[0] = HYTECH_RF_TTPMS_T1_ro_fromS(p_unpacked_msg->RF_TTPMS_T1_ro); 
                _ttpms_data.rf_ttpms.temp_data[1] = HYTECH_RF_TTPMS_T2_ro_fromS(p_unpacked_msg->RF_TTPMS_T2_ro);
                _ttpms_data.rf_ttpms.temp_data[2] = HYTECH_RF_TTPMS_T3_ro_fromS(p_unpacked_msg->RF_TTPMS_T3_ro);
                _ttpms_data.rf_ttpms.temp_data[3] = HYTECH_RF_TTPMS_T4_ro_fromS(p_unpacked_msg->RF_TTPMS_T4_ro);
                break;
            }
            case RF_TTPMS_3_CANID:
            {
                RF_TTPMS_3_t unpacked_msg;
                Unpack_RF_TTPMS_3_hytech(&unpacked_msg, msg.buf, msg.len);
                RF_TTPMS_3_t* p_unpacked_msg = &unpacked_msg;
                _ttpms_data.rf_ttpms.temp_data[4] = HYTECH_RF_TTPMS_T5_ro_fromS(p_unpacked_msg->RF_TTPMS_T5_ro); 
                _ttpms_data.rf_ttpms.temp_data[5] = HYTECH_RF_TTPMS_T6_ro_fromS(p_unpacked_msg->RF_TTPMS_T6_ro);
                _ttpms_data.rf_ttpms.temp_data[6] = HYTECH_RF_TTPMS_T7_ro_fromS(p_unpacked_msg->RF_TTPMS_T7_ro);
                _ttpms_data.rf_ttpms.temp_data[7] = HYTECH_RF_TTPMS_T8_ro_fromS(p_unpacked_msg->RF_TTPMS_T8_ro);
                break;
            }
            case RF_TTPMS_4_CANID:
            {
                RF_TTPMS_4_t unpacked_msg;
                Unpack_RF_TTPMS_4_hytech(&unpacked_msg, msg.buf, msg.len);
                RF_TTPMS_4_t* p_unpacked_msg = &unpacked_msg;
                _ttpms_data.rf_ttpms.temp_data[8] = HYTECH_RF_TTPMS_T9_ro_fromS(p_unpacked_msg->RF_TTPMS_T9_ro); 
                _ttpms_data.rf_ttpms.temp_data[9] = HYTECH_RF_TTPMS_T10_ro_fromS(p_unpacked_msg->RF_TTPMS_T10_ro);
                _ttpms_data.rf_ttpms.temp_data[10] = HYTECH_RF_TTPMS_T11_ro_fromS(p_unpacked_msg->RF_TTPMS_T11_ro);
                _ttpms_data.rf_ttpms.temp_data[11] = HYTECH_RF_TTPMS_T12_ro_fromS(p_unpacked_msg->RF_TTPMS_T12_ro);
                break;
            }
            case RF_TTPMS_5_CANID:
            {
                RF_TTPMS_5_t unpacked_msg;
                Unpack_RF_TTPMS_5_hytech(&unpacked_msg, msg.buf, msg.len);
                RF_TTPMS_5_t* p_unpacked_msg = &unpacked_msg;
                _ttpms_data.rf_ttpms.temp_data[12] = HYTECH_RF_TTPMS_T13_ro_fromS(p_unpacked_msg->RF_TTPMS_T13_ro); 
                _ttpms_data.rf_ttpms.temp_data[13] = HYTECH_RF_TTPMS_T14_ro_fromS(p_unpacked_msg->RF_TTPMS_T14_ro);
                _ttpms_data.rf_ttpms.temp_data[14] = HYTECH_RF_TTPMS_T15_ro_fromS(p_unpacked_msg->RF_TTPMS_T15_ro);
                _ttpms_data.rf_ttpms.temp_data[15] = HYTECH_RF_TTPMS_T16_ro_fromS(p_unpacked_msg->RF_TTPMS_T16_ro);
                break;
            }
        }
        
        // Rear left TTPMS
        {
            case LR_TTPMS_1_CANID:
            {
                LR_TTPMS_1_t unpacked_msg;
                Unpack_LR_TTPMS_1_hytech(&unpacked_msg, msg.buf, msg.len);
                LR_TTPMS_1_t* p_unpacked_msg = &unpacked_msg;
                _ttpms_data.lr_ttpms.bat_voltage = p_unpacked_msg->LR_TTPMS_BAT_V;
                _ttpms_data.lr_ttpms.pressure = p_unpacked_msg->LR_TTPMS_P_ro;
                _ttpms_data.lr_ttpms.pressure = p_unpacked_msg->LR_TTPMS_P_GAUGE;   
                break;
            }
            case LR_TTPMS_2_CANID:
            {
                LR_TTPMS_2_t unpacked_msg;
                Unpack_LR_TTPMS_2_hytech(&unpacked_msg, msg.buf, msg.len);
                LR_TTPMS_2_t* p_unpacked_msg = &unpacked_msg;
                _ttpms_data.lr_ttpms.temp_data[0] = HYTECH_LR_TTPMS_T1_ro_fromS(p_unpacked_msg->LR_TTPMS_T1_ro); // unpack struct pointer OR just use struct attribute?
                _ttpms_data.lr_ttpms.temp_data[1] = HYTECH_LR_TTPMS_T2_ro_fromS(p_unpacked_msg->LR_TTPMS_T2_ro);
                _ttpms_data.lr_ttpms.temp_data[2] = HYTECH_LR_TTPMS_T3_ro_fromS(p_unpacked_msg->LR_TTPMS_T3_ro);
                _ttpms_data.lr_ttpms.temp_data[3] = HYTECH_LR_TTPMS_T4_ro_fromS(p_unpacked_msg->LR_TTPMS_T4_ro);
                break;
            }
            case LR_TTPMS_3_CANID:
            {
                LR_TTPMS_3_t unpacked_msg;
                Unpack_LR_TTPMS_3_hytech(&unpacked_msg, msg.buf, msg.len);
                LR_TTPMS_3_t* p_unpacked_msg = &unpacked_msg;
                _ttpms_data.lr_ttpms.temp_data[4] = HYTECH_LR_TTPMS_T5_ro_fromS(p_unpacked_msg->LR_TTPMS_T5_ro); 
                _ttpms_data.lr_ttpms.temp_data[5] = HYTECH_LR_TTPMS_T6_ro_fromS(p_unpacked_msg->LR_TTPMS_T6_ro);
                _ttpms_data.lr_ttpms.temp_data[6] = HYTECH_LR_TTPMS_T7_ro_fromS(p_unpacked_msg->LR_TTPMS_T7_ro);
                _ttpms_data.lr_ttpms.temp_data[7] = HYTECH_LR_TTPMS_T8_ro_fromS(p_unpacked_msg->LR_TTPMS_T8_ro);
                break;
            }
            case LR_TTPMS_4_CANID:
            {
                LR_TTPMS_4_t unpacked_msg;
                Unpack_LR_TTPMS_4_hytech(&unpacked_msg, msg.buf, msg.len);
                LR_TTPMS_4_t* p_unpacked_msg = &unpacked_msg;
                _ttpms_data.lr_ttpms.temp_data[8] = HYTECH_LR_TTPMS_T9_ro_fromS(p_unpacked_msg->LR_TTPMS_T9_ro); 
                _ttpms_data.lr_ttpms.temp_data[9] = HYTECH_LR_TTPMS_T10_ro_fromS(p_unpacked_msg->LR_TTPMS_T10_ro);
                _ttpms_data.lr_ttpms.temp_data[10] = HYTECH_LR_TTPMS_T11_ro_fromS(p_unpacked_msg->LR_TTPMS_T11_ro);
                _ttpms_data.lr_ttpms.temp_data[11] = HYTECH_LR_TTPMS_T12_ro_fromS(p_unpacked_msg->LR_TTPMS_T12_ro);
                break;
            }
            case LR_TTPMS_5_CANID:
            {
                LR_TTPMS_5_t unpacked_msg;
                Unpack_LR_TTPMS_5_hytech(&unpacked_msg, msg.buf, msg.len);
                LR_TTPMS_5_t* p_unpacked_msg = &unpacked_msg;
                _ttpms_data.lr_ttpms.temp_data[12] = HYTECH_LR_TTPMS_T13_ro_fromS(p_unpacked_msg->LR_TTPMS_T13_ro); 
                _ttpms_data.lr_ttpms.temp_data[13] = HYTECH_LR_TTPMS_T14_ro_fromS(p_unpacked_msg->LR_TTPMS_T14_ro);
                _ttpms_data.lr_ttpms.temp_data[14] = HYTECH_LR_TTPMS_T15_ro_fromS(p_unpacked_msg->LR_TTPMS_T15_ro);
                _ttpms_data.lr_ttpms.temp_data[15] = HYTECH_LR_TTPMS_T16_ro_fromS(p_unpacked_msg->LR_TTPMS_T16_ro);
                break;
            }
        }

        // Rear right TTPMS
        {    
            case RR_TTPMS_1_CANID:
            {
                RR_TTPMS_1_t unpacked_msg;
                Unpack_RR_TTPMS_1_hytech(&unpacked_msg, msg.buf, msg.len);
                RR_TTPMS_1_t* p_unpacked_msg = &unpacked_msg;
                _ttpms_data.rr_ttpms.bat_voltage = p_unpacked_msg->RR_TTPMS_BAT_V;
                _ttpms_data.rr_ttpms.pressure = p_unpacked_msg->RR_TTPMS_P_ro;
                _ttpms_data.rr_ttpms.pressure = p_unpacked_msg->RR_TTPMS_P_GAUGE;   
                break;
            }
            case RR_TTPMS_2_CANID:
            {
                RR_TTPMS_2_t unpacked_msg;
                Unpack_RR_TTPMS_2_hytech(&unpacked_msg, msg.buf, msg.len);
                RR_TTPMS_2_t* p_unpacked_msg = &unpacked_msg;
                _ttpms_data.rr_ttpms.temp_data[0] = HYTECH_RR_TTPMS_T1_ro_fromS(p_unpacked_msg->RR_TTPMS_T1_ro); 
                _ttpms_data.rr_ttpms.temp_data[1] = HYTECH_RR_TTPMS_T2_ro_fromS(p_unpacked_msg->RR_TTPMS_T2_ro);
                _ttpms_data.rr_ttpms.temp_data[2] = HYTECH_RR_TTPMS_T3_ro_fromS(p_unpacked_msg->RR_TTPMS_T3_ro);
                _ttpms_data.rr_ttpms.temp_data[3] = HYTECH_RR_TTPMS_T4_ro_fromS(p_unpacked_msg->RR_TTPMS_T4_ro);

                break;
            }
            case RR_TTPMS_3_CANID:
            {
                RR_TTPMS_3_t unpacked_msg;
                Unpack_RR_TTPMS_3_hytech(&unpacked_msg, msg.buf, msg.len);
                RR_TTPMS_3_t* p_unpacked_msg = &unpacked_msg;
                _ttpms_data.rr_ttpms.temp_data[4] = HYTECH_RR_TTPMS_T5_ro_fromS(p_unpacked_msg->RR_TTPMS_T5_ro); 
                _ttpms_data.rr_ttpms.temp_data[5] = HYTECH_RR_TTPMS_T6_ro_fromS(p_unpacked_msg->RR_TTPMS_T6_ro);
                _ttpms_data.rr_ttpms.temp_data[6] = HYTECH_RR_TTPMS_T7_ro_fromS(p_unpacked_msg->RR_TTPMS_T7_ro);
                _ttpms_data.rr_ttpms.temp_data[7] = HYTECH_RR_TTPMS_T8_ro_fromS(p_unpacked_msg->RR_TTPMS_T8_ro);
                break;
            }
            case RR_TTPMS_4_CANID:
            {
                RR_TTPMS_4_t unpacked_msg;
                Unpack_RR_TTPMS_4_hytech(&unpacked_msg, msg.buf, msg.len);
                RR_TTPMS_4_t* p_unpacked_msg = &unpacked_msg;
                _ttpms_data.rr_ttpms.temp_data[8] = HYTECH_RR_TTPMS_T9_ro_fromS(p_unpacked_msg->RR_TTPMS_T9_ro); 
                _ttpms_data.rr_ttpms.temp_data[9] = HYTECH_RR_TTPMS_T10_ro_fromS(p_unpacked_msg->RR_TTPMS_T10_ro);
                _ttpms_data.rr_ttpms.temp_data[10] = HYTECH_RR_TTPMS_T11_ro_fromS(p_unpacked_msg->RR_TTPMS_T11_ro);
                _ttpms_data.rr_ttpms.temp_data[11] = HYTECH_RR_TTPMS_T12_ro_fromS(p_unpacked_msg->RR_TTPMS_T12_ro);
                break;
            }
            case RR_TTPMS_5_CANID:
            {
                RR_TTPMS_5_t unpacked_msg;
                Unpack_RR_TTPMS_5_hytech(&unpacked_msg, msg.buf, msg.len);
                RR_TTPMS_5_t* p_unpacked_msg = &unpacked_msg;
                _ttpms_data.rr_ttpms.temp_data[12] = HYTECH_RR_TTPMS_T13_ro_fromS(p_unpacked_msg->RR_TTPMS_T13_ro); 
                _ttpms_data.rr_ttpms.temp_data[13] = HYTECH_RR_TTPMS_T14_ro_fromS(p_unpacked_msg->RR_TTPMS_T14_ro);
                _ttpms_data.rr_ttpms.temp_data[14] = HYTECH_RR_TTPMS_T15_ro_fromS(p_unpacked_msg->RR_TTPMS_T15_ro);
                _ttpms_data.rr_ttpms.temp_data[15] = HYTECH_RR_TTPMS_T16_ro_fromS(p_unpacked_msg->RR_TTPMS_T16_ro);
                break;
            }
        } 
        
    }

}
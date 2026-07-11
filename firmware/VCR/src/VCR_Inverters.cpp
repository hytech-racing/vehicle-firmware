#include "VCR_Inverters.h"


InverterInterface fl_inverter_interface(INV1_CONTROL_WORD_CANID,
                                INV1_CONTROL_INPUT_CANID,
                                INV1_CONTROL_PARAMETER_CANID,
                                {.MINIMUM_HV_VOLTAGE = VCRSystems::INVERTER_MINIMUM_HV_VOLTAGE} //NOLINT
);

InverterInterface fr_inverter_interface(INV2_CONTROL_WORD_CANID,
                                INV2_CONTROL_INPUT_CANID,
                                INV2_CONTROL_PARAMETER_CANID,
                                {.MINIMUM_HV_VOLTAGE = VCRSystems::INVERTER_MINIMUM_HV_VOLTAGE} //NOLINT
);

InverterInterface rl_inverter_interface(INV3_CONTROL_WORD_CANID,
                                INV3_CONTROL_INPUT_CANID,
                                INV3_CONTROL_PARAMETER_CANID,
                                {.MINIMUM_HV_VOLTAGE = VCRSystems::INVERTER_MINIMUM_HV_VOLTAGE} //NOLINT
);

InverterInterface rr_inverter_interface(INV4_CONTROL_WORD_CANID,
                                INV4_CONTROL_INPUT_CANID,
                                INV4_CONTROL_PARAMETER_CANID,
                                {.MINIMUM_HV_VOLTAGE = VCRSystems::INVERTER_MINIMUM_HV_VOLTAGE} //NOLINT
);
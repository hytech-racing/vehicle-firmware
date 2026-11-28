#ifndef FIRMWARE_VERSION_CAN_HPP
#define FIRMWARE_VERSION_CAN_HPP

/* Generated at build time by pio-git-hash-gen */
#include "device_fw_version.h"

namespace FirmwareVersionCAN
{
    /**
     * @brief Builds a <BOARD>_FW_VERSION CAN message holding this build's git hash and whether it was built from main/master.
     * @tparam FWVersionMsg_t Any HT_CAN <BOARD>_FW_VERSION_t struct (they all share the same signals)
     * @note Reads FW_PROJECT_IS_MAIN_OR_MASTER instead of device_status_t::project_on_main_or_master, because that field
     *       is a bool and turns the generator's "testing branch" value (2) into true.
     */
    template <typename FWVersionMsg_t>
    FWVersionMsg_t makeFirmwareVersionMsg()
    {
        FWVersionMsg_t msg = {};
        msg.fw_git_hash = device_status_t::firmware_version;
        msg.fw_on_main_or_master = (FW_PROJECT_IS_MAIN_OR_MASTER == 1);
        return msg;
    }
} // namespace FirmwareVersionCAN

#endif // FIRMWARE_VERSION_CAN_HPP

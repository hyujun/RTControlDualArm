/**
 * @file Ecat_Elmo.h
 * @date 2018-11-15
 * @author Junho Park
 */

#pragma once

#include "ecat/core/CiA402Device.h"

namespace hyuEcat {

class EcatElmo : public CiA402Device
{
public:
    EcatElmo() : CiA402Device(Elmo_VendorID, Elmo_ProductCode) {
        dc_enabled_         = true;
        dc_assign_activate_ = 0x0300;
        dc_sync0_shift_     = 400000;
    }
    ~EcatElmo() override = default;

    int Elmo_DeviceState() const noexcept { return getDeviceStateAsInt(); }

    void setupSDOs(ec_slave_config_t* config) override
    {
        if (isHomingEnable()) {
            ecrt_slave_config_sdo32(config, 0x607C, 0, getHomingOffset());
            ecrt_slave_config_sdo8 (config, 0x6098, 0, getHomingMethod());
            ecrt_slave_config_sdo32(config, 0x6099, 2, getHomingSpeed()); // SUBINDEX_HOMING_LOWSPEED = 0x02
            ecrt_slave_config_sdo16(config, 0x2020, 1, getHomingCurrentLimit());
        }
    }

    void syncDomainMap(const std::vector<uint8_t*>& ptrs) override
    {
        if (ptrs.size() < 9) return;
        p_target_torque_        = ptrs[0]; // 0x6071 S16
        p_controlword_          = ptrs[1]; // 0x6040 U16
        p_max_torque_           = ptrs[2]; // 0x6072 U16
        p_mode_of_operation_    = ptrs[3]; // 0x6060 S8
        p_velocity_             = ptrs[4]; // 0x6069 S32
        p_position_             = ptrs[5]; // 0x6064 S32
        p_torque_               = ptrs[6]; // 0x6077 S16
        p_statusword_           = ptrs[7]; // 0x6041 U16
        p_mode_of_op_display_   = ptrs[8]; // 0x6061 S8
    }

    void updateRx() override
    {
        velocity_ = EC_READ_S32(p_velocity_);
        position_ = EC_READ_S32(p_position_);
        torque_   = EC_READ_S16(p_torque_);
        const uint16_t sw = EC_READ_U16(p_statusword_);
        mode_of_operation_display_ = EC_READ_S8(p_mode_of_op_display_);

        if (sw != last_status_word_) {
            state_ = deviceState(sw);
        }
        status_word_ = sw;

        initialized_ = (state_ == STATE_OPERATION_ENABLED &&
                        last_state_ == STATE_OPERATION_ENABLED)
                     || (mode_of_operation_display_ == MODE_HOMING);
        homing_ready_ = (state_ == STATE_HOMING_NOT_START &&
                         last_state_ == STATE_HOMING_NOT_START);
        last_status_word_ = sw;
        last_state_       = state_;
    }

    void updateTx() override
    {
        EC_WRITE_S16(p_target_torque_, target_torque_);

        control_word_ = EC_READ_U16(p_controlword_);
        control_word_ = transition(state_, control_word_);
        EC_WRITE_U16(p_controlword_, control_word_);

        EC_WRITE_U16(p_max_torque_,        max_torque_);
        EC_WRITE_S8 (p_mode_of_operation_, mode_of_operation_);
    }

    const ec_sync_info_t*      syncs()    override { return Elmo_syncs; }
    size_t                     syncSize() override { return sizeof(Elmo_syncs) / sizeof(ec_sync_info_t); }
    const ec_pdo_entry_info_t* channels() override { return Elmo_pdo_entries; }
    void domains(DomainMap& d) const override { d = domains_; }

private:
    uint8_t* p_target_torque_      = nullptr;
    uint8_t* p_controlword_        = nullptr;
    uint8_t* p_max_torque_         = nullptr;
    uint8_t* p_mode_of_operation_  = nullptr;
    uint8_t* p_velocity_           = nullptr;
    uint8_t* p_position_           = nullptr;
    uint8_t* p_torque_             = nullptr;
    uint8_t* p_statusword_         = nullptr;
    uint8_t* p_mode_of_op_display_ = nullptr;

    DomainMap domains_ = { {0, {0,1,2,3,4,5,6,7,8}} };
};

} /* namespace hyuEcat */

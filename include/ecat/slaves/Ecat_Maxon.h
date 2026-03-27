/**
 * @file Ecat_Maxon.h
 * @date 2018-11-15
 * @author Junho Park
 */

#pragma once

#include "ecat/core/CiA402Device.h"

namespace hyuEcat {

class EcatMaxon : public CiA402Device
{
public:
    EcatMaxon() : CiA402Device(Maxon_VenderID, Maxon_ProductCode) {
        dc_enabled_         = true;
        dc_assign_activate_ = 0x0300;
        dc_sync0_shift_     = 400000;
    }
    ~EcatMaxon() override = default;

    int Maxon_DeviceState() const noexcept { return getDeviceStateAsInt(); }

    // Pre-configured homing parameters for Maxon (originally in NVRAM or set manually)
    int32_t  HomingOffset       (int pos) const noexcept { return m_HomingOffset[pos];        }
    int8_t   HomingMethod       (int pos) const noexcept { return m_HomingMethod[pos];        }
    uint32_t HomingSpeed        (int pos) const noexcept { return m_HomingSpeed[pos];         }
    uint16_t HomingCurrentLimit (int pos) const noexcept { return m_HomingCurrentLimit[pos];  }

    void setupSDOs(ec_slave_config_t* config) override
    {
        // Custom Maxon SDO configuration can be placed here if needed
        // Original code did not implement Master::addSlaveWithHoming for Maxon,
        // homing SDOs were only downloaded for Elmo.
    }

    void syncDomainMap(const std::vector<uint8_t*>& ptrs) override
    {
        if (ptrs.size() < 10) return;
        p_target_position_   = ptrs[0]; // 0x607A S32
        p_target_velocity_   = ptrs[1]; // 0x60FF S32
        p_target_torque_     = ptrs[2]; // 0x6071 S16
        p_controlword_       = ptrs[3]; // 0x6040 U16
        p_mode_of_operation_ = ptrs[4]; // 0x6060 S8
        p_position_          = ptrs[5]; // 0x6064 S32
        p_velocity_          = ptrs[6]; // 0x606C S32
        p_torque_            = ptrs[7]; // 0x6077 S16
        p_statusword_        = ptrs[8]; // 0x6041 U16
        p_mode_of_op_display_= ptrs[9]; // 0x6061 S8
    }

    void updateRx() override
    {
        position_ = EC_READ_S32(p_position_);
        velocity_ = EC_READ_S32(p_velocity_);
        torque_   = EC_READ_S16(p_torque_);
        const uint16_t sw = EC_READ_U16(p_statusword_);
        mode_of_operation_display_ = EC_READ_S8(p_mode_of_op_display_);

        if (sw != last_status_word_) {
            state_ = deviceState(sw);
        }
        status_word_ = sw;

        initialized_ = (state_ == STATE_OPERATION_ENABLED &&
                        last_state_ == STATE_OPERATION_ENABLED);
        homing_ready_ = (state_ == STATE_HOMING_NOT_START &&
                         last_state_ == STATE_HOMING_NOT_START);
        last_status_word_ = sw;
        last_state_       = state_;
    }

    void updateTx() override
    {
        EC_WRITE_S32(p_target_position_, target_position_);
        EC_WRITE_S32(p_target_velocity_, target_velocity_);
        EC_WRITE_S16(p_target_torque_,   target_torque_);

        control_word_ = EC_READ_U16(p_controlword_);
        control_word_ = transition(state_, control_word_);
        EC_WRITE_U16(p_controlword_,       control_word_);
        EC_WRITE_S8 (p_mode_of_operation_, mode_of_operation_);
    }

    const ec_sync_info_t*      syncs()    override { return &Maxon_syncs[0]; }
    size_t                     syncSize() override { return sizeof(Maxon_syncs) / sizeof(ec_sync_info_t); }
    const ec_pdo_entry_info_t* channels() override { return Maxon_pdo_entries; }
    void domains(DomainMap& d) const override { d = domains_; }

private:
    uint8_t* p_target_position_   = nullptr;
    uint8_t* p_target_velocity_   = nullptr;
    uint8_t* p_target_torque_     = nullptr;
    uint8_t* p_controlword_       = nullptr;
    uint8_t* p_mode_of_operation_ = nullptr;
    uint8_t* p_position_          = nullptr;
    uint8_t* p_velocity_          = nullptr;
    uint8_t* p_torque_            = nullptr;
    uint8_t* p_statusword_        = nullptr;
    uint8_t* p_mode_of_op_display_= nullptr;

    DomainMap domains_ = { {0, {0,1,2,3,4,5,6,7,8,9}} };

    int32_t  m_HomingOffset      [3] = { -136490, 75727, 120023 };
    int8_t   m_HomingMethod      [3] = { -4, -3, -3 };
    uint32_t m_HomingSpeed       [3] = { 17000, 17000, 17000 };
    uint16_t m_HomingCurrentLimit[3] = { 700, 700, 700 };
};

} /* namespace hyuEcat */

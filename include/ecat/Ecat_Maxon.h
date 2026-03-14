/**
 * @file Ecat_Maxon.h
 * @brief EtherCAT driver for Maxon EPOS4-EtherCAT (CiA 402)
 * @date 2018-11-15
 * @author Junho Park
 */

#pragma once

#include "ecat/Ecat_Slave.h"

//#define _MAXON_DEBUG_

namespace hyuEcat {

/**
 * @brief EtherCAT driver for Maxon EPOS4-EtherCAT 50/15 (CiA 402)
 * @version 1.2.0
 */
class EcatMaxon : public Slave
{
public:
    EcatMaxon() : Slave(Maxon_VenderID, Maxon_ProductCode) {}
    ~EcatMaxon() override = default;

    bool initialized()       const noexcept { return initialized_; }
    bool isHoming()          const noexcept { return homing_; }
    int  Maxon_DeviceState() const noexcept { return static_cast<int>(state_); }

    void writeTorque(int16_t torque) noexcept { target_torque_ = torque; }

    // Pre-configured homing parameters (per-axis, hardcoded from commissioning)
    int32_t  HomingOffset       (int pos) const noexcept { return m_HomingOffset[pos];       }
    int8_t   HomingMethod       (int pos) const noexcept { return m_HomingMethod[pos];        }
    uint32_t HomingSpeed        (int pos) const noexcept { return m_HomingSpeed[pos];         }
    uint16_t HomingCurrentLimit (int pos) const noexcept { return m_HomingCurrentLimit[pos];  }

    /**
     * @brief Returns a string literal for the current device state.
     * RT-safe: O(1) array lookup, no heap allocation, no copy.
     */
    const char* GetDevState() const noexcept
    {
        const int s = static_cast<int>(state_);
        return (s >= 0 && s < kNumStates) ? kStateStr[s] : "Unknown";
    }

    // ---- PDO processing (called from 1 kHz RT loop) ----
    void processData(size_t index, uint8_t* domain_address) override
    {
        switch (index)
        {
        // RxPDO
        case 0: EC_WRITE_S32(domain_address, target_position_);   break;
        case 1: EC_WRITE_S32(domain_address, target_velocity_);   break;
        case 2: EC_WRITE_S16(domain_address, target_torque_);     break;
        case 3:
            control_word_ = EC_READ_U16(domain_address);
            control_word_ = transition(state_, control_word_);
            EC_WRITE_U16(domain_address, control_word_);
            break;
        case 4: EC_WRITE_S8(domain_address, mode_of_operation_);  break;
        // TxPDO
        case 5: position_                  = EC_READ_S32(domain_address); break;
        case 6: velocity_                  = EC_READ_S32(domain_address); break;
        case 7: torque_                    = EC_READ_S16(domain_address); break;
        case 8:
            status_word_ = EC_READ_U16(domain_address);
            state_       = deviceState(status_word_);
            break;
        case 9: mode_of_operation_display_ = EC_READ_S8(domain_address);  break;
        default:
#if defined(_MAXON_DEBUG_)
            fprintf(stderr, "WARNING. Maxon EPOS4: PDO index %zu out of range.\n", index);
#endif
            break;
        }

        if (index == 8) // last status entry: update state flags
        {
            if (status_word_ != last_status_word_) {
                state_ = deviceState(status_word_);
#if defined(_MAXON_DEBUG_)
                if (state_ != last_state_)
                    fprintf(stdout, "MAXON pos:%d state:%s\n", slave_position, GetDevState());
#endif
            }
            initialized_ = (state_ == STATE_OPERATION_ENABLED &&
                            last_state_ == STATE_OPERATION_ENABLED);
            homing_ready_ = (state_ == STATE_HOMING_NOT_START &&
                             last_state_ == STATE_HOMING_NOT_START);
            last_status_word_ = status_word_;
            last_state_       = state_;
        }
    }

    // ---- PDO / sync descriptors ----
    const ec_sync_info_t*      syncs()    override { return &Maxon_syncs[0]; }
    size_t                     syncSize() override { return sizeof(Maxon_syncs) / sizeof(ec_sync_info_t); }
    const ec_pdo_entry_info_t* channels() override { return Maxon_pdo_entries; }
    void domains(DomainMap& d) const override { d = domains_; }

    // ---- PDO data (public for direct RT access) ----
    int32_t  target_position_           = 0;
    int32_t  target_velocity_           = 0;
    int16_t  target_torque_             = 0;
    uint16_t max_torque_                = 1800;
    uint16_t control_word_              = 0;
    int8_t   mode_of_operation_         = MODE_CYCLIC_SYNC_TORQUE;

    int32_t  position_                  = 0;
    int32_t  velocity_                  = 0;
    int16_t  torque_                    = 0;
    uint16_t status_word_               = 0;
    int8_t   mode_of_operation_display_ = 0;

    enum ModeOfOperation : int8_t
    {
        MODE_NO_MODE               = 0,
        MODE_PROFILED_POSITION     = 1,
        MODE_PROFILED_VELOCITY     = 3,
        MODE_PROFILED_TORQUE       = 4,
        MODE_HOMING                = 6,
        MODE_INTERPOLATED_POSITION = 7,
        MODE_CYCLIC_SYNC_POSITION  = 8,
        MODE_CYCLIC_SYNC_VELEOCITY = 9,
        MODE_CYCLIC_SYNC_TORQUE    = 10
    };

private:

    DomainMap domains_ = { {0, {0,1,2,3,4,5,6,7,8,9}} };

    // ---- CiA 402 device state machine ----

    enum DeviceState : int
    {
        STATE_UNDEFINED                   = 0,
        STATE_START                       = 1,
        // 2 unused
        STATE_NOT_READY_TO_SWITCH_ON      = 3,
        STATE_SWITCH_ON_DISABLED          = 4,
        STATE_READY_TO_SWITCH_ON          = 5,
        STATE_SWITCH_ON                   = 6,
        STATE_OPERATION_ENABLED           = 7,
        STATE_QUICK_STOP_ACTIVE           = 8,
        STATE_FAULT_REACTION_ACTIVE       = 9,
        STATE_FAULT                       = 10,
        STATE_HOMING_PROGRESS             = 11,
        STATE_HOMING_NOT_START            = 12,
        STATE_HOMING_ATTAINED_NOT_REACHED = 13,
        STATE_HOMING_COMPLITE             = 14,
        STATE_HOMING_ERROR                = 15,
        STATE_HOMING_UNDIFINED            = 16
    };

    static constexpr int kNumStates = 17;
    static constexpr const char* kStateStr[kNumStates] = {
        "Undefined",                   // 0
        "Start",                       // 1
        "Undefined",                   // 2 (unused)
        "Not Ready to Switch On",      // 3
        "Switch On Disabled",          // 4
        "Ready to Switch On",          // 5
        "Switch On",                   // 6
        "Operation Enabled",           // 7
        "Quick Stop Active",           // 8
        "Fault Reaction Active",       // 9
        "Fault",                       // 10
        "Homing Progress",             // 11
        "Homing Not Start",            // 12
        "Homing Attained Not Reached", // 13
        "Homing Finished",             // 14
        "Homing Error",                // 15
        "Homing Undefined"             // 16
    };

    DeviceState deviceState(uint16_t sw) noexcept
    {
        if      ((sw & 0x4F) == 0x00) return STATE_NOT_READY_TO_SWITCH_ON;
        else if ((sw & 0x4F) == 0x40) return STATE_SWITCH_ON_DISABLED;
        else if ((sw & 0x6F) == 0x21) return STATE_READY_TO_SWITCH_ON;
        else if ((sw & 0x6F) == 0x23) return STATE_SWITCH_ON;
        else if ((sw & 0x6F) == 0x27) {
            if (mode_of_operation_display_ == MODE_HOMING) {
                const uint8_t h = static_cast<uint8_t>(sw >> 8) & 0x34u;
                if      (h == 0x00) return STATE_HOMING_PROGRESS;
                else if (h == 0x04) return STATE_HOMING_NOT_START;
                else if (h == 0x10) return STATE_HOMING_ATTAINED_NOT_REACHED;
                else if (h == 0x14) { homing_ = true; return STATE_HOMING_COMPLITE; }
                else if (h == 0x20 || h == 0x24) return STATE_HOMING_ERROR;
                return STATE_HOMING_UNDIFINED;
            }
            return STATE_OPERATION_ENABLED;
        }
        else if ((sw & 0x6F) == 0x07) return STATE_QUICK_STOP_ACTIVE;
        else if ((sw & 0x4F) == 0x0F) return STATE_FAULT_REACTION_ACTIVE;
        else if ((sw & 0x4F) == 0x08) return STATE_FAULT;
        return STATE_UNDEFINED;
    }

    uint16_t transition(DeviceState state, uint16_t cw) const noexcept
    {
        switch (state)
        {
        case STATE_START:
        case STATE_NOT_READY_TO_SWITCH_ON:       return cw;
        case STATE_SWITCH_ON_DISABLED:           return (cw & 0x7Eu) | 0x06u;
        case STATE_READY_TO_SWITCH_ON:           return (cw & 0x77u) | 0x07u;
        case STATE_SWITCH_ON:                    return (cw & 0x70u) | 0x0Fu;
        case STATE_OPERATION_ENABLED:            return 0x0Fu;
        case STATE_QUICK_STOP_ACTIVE:            return (cw & 0x7Fu) | 0x0Fu;
        case STATE_FAULT_REACTION_ACTIVE:        return cw;
        case STATE_FAULT:                        return cw | 0x80u;
        case STATE_HOMING_PROGRESS:
        case STATE_HOMING_ATTAINED_NOT_REACHED:
        case STATE_HOMING_UNDIFINED:             return (cw & 0x1Fu) | 0x1Fu;
        case STATE_HOMING_NOT_START:             return homing_ready_ ? ((cw & 0x1Fu) | 0x1Fu) : cw;
        case STATE_HOMING_COMPLITE:
        case STATE_HOMING_ERROR:                 return (cw & 0x0Fu) | 0x0Fu;
        default:                                 return cw;
        }
    }

    int         last_status_word_ = -1;
    DeviceState last_state_       = STATE_START;
    DeviceState state_            = STATE_START;

    bool initialized_  = false;
    bool homing_       = false;
    bool homing_ready_ = false;

    // Commissioning-time homing parameters (3-axis wrist)
    int32_t  m_HomingOffset      [3] = { -136490, 75727, 120023 };
    int8_t   m_HomingMethod      [3] = { -4, -3, -3 };
    uint32_t m_HomingSpeed       [3] = { 17000, 17000, 17000 };
    uint16_t m_HomingCurrentLimit[3] = { 700, 700, 700 };
};

constexpr const char* EcatMaxon::kStateStr[EcatMaxon::kNumStates];

} /* namespace hyuEcat */


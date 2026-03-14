/**
 * @file Ecat_KistFinger.h
 * @brief EtherCAT driver for KIST anthropomorphic hand finger motors (4-axis)
 * @date 2018-11-15
 * @author Junho Park
 */

#pragma once

#include "ecat/Ecat_Slave.h"

//#define _KIST_FINGER_DEBUG_

namespace hyuEcat {

/**
 * @brief EtherCAT driver for KIST hand finger motor controller (4 motors)
 * @version 1.1.0
 */
class EcatKistFinger : public Slave
{
public:
    EcatKistFinger() : Slave(KistHand_VendorID, KistHand_ProductCode) {}
    ~EcatKistFinger() override = default;

    bool initialized() const noexcept { return initialized_; }
    int  GetDeviceState() const noexcept { return static_cast<int>(state_); }

    /**
     * @brief Returns a string literal for the current device state.
     * RT-safe: O(1) array lookup, no heap allocation, no copy.
     */
    const char* GetDevstr() const noexcept
    {
        const int s = static_cast<int>(state_);
        return (s >= 0 && s < kNumStates) ? kStateStr[s] : "Unknown";
    }

    void SetVelocityTarget(const int16_t* velocity) noexcept
    {
        for (int i = 0; i < 4; ++i) command[i] = velocity[i];
    }

    // ---- PDO processing (called from 1 kHz RT loop) ----
    void processData(size_t index, uint8_t* domain_address) override
    {
        switch (index)
        {
        // RxPDO
        case 0: EC_WRITE_S16(domain_address, command[0]); break;
        case 1: EC_WRITE_S16(domain_address, command[1]); break;
        case 2: EC_WRITE_S16(domain_address, command[2]); break;
        case 3: EC_WRITE_S16(domain_address, command[3]); break;
        // TxPDO
        case 4:  MotorPos[0] = EC_READ_S16(domain_address); break;
        case 5:  MotorPos[1] = EC_READ_S16(domain_address); break;
        case 6:  MotorPos[2] = EC_READ_S16(domain_address); break;
        case 7:  MotorPos[3] = EC_READ_S16(domain_address); break;
        case 8:  MotorVel[0] = EC_READ_S16(domain_address); break;
        case 9:  MotorVel[1] = EC_READ_S16(domain_address); break;
        case 10: MotorVel[2] = EC_READ_S16(domain_address); break;
        case 11: MotorVel[3] = EC_READ_S16(domain_address); break;
        case 12: MotorStatus = EC_READ_S16(domain_address); break;
        case 13: case 14: case 15: case 16: case 17: case 18: break; // reserved
        case 19: timer_tick = EC_READ_U16(domain_address); break;
        default:
#if defined(_KIST_FINGER_DEBUG_)
            fprintf(stderr, "WARNING. KIST Finger: PDO index %zu out of range.\n", index);
#endif
            break;
        }
    }

    // ---- PDO / sync descriptors ----
    const ec_sync_info_t*      syncs()    override { return &KistHand_syncs[0]; }
    size_t                     syncSize() override { return sizeof(KistHand_syncs) / sizeof(ec_sync_info_t); }
    const ec_pdo_entry_info_t* channels() override { return KistHand_pdo_entries; }
    void domains(DomainMap& d) const override { d = domains_; }

    // ---- PDO data (public for direct RT access) ----
    int16_t  command    [4] = {0};
    int16_t  MotorPos   [4] = {0};
    int16_t  MotorVel   [4] = {0};
    int16_t  MotorInput [4] = {0};
    int16_t  MotorTorque[2] = {0};
    int16_t  MotorStatus    = 0;
    uint16_t timer_tick     = 0;

private:

    DomainMap domains_ = { {0, {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19}} };

    enum DeviceState : int
    {
        STATE_UNDEFINED                   = 0,
        STATE_START                       = 1,
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
        "Undefined",                   // 2
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

    DeviceState state_       = STATE_START;
    bool        initialized_ = false;
};

constexpr const char* EcatKistFinger::kStateStr[EcatKistFinger::kNumStates];

} /* namespace hyuEcat */


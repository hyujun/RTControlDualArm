/**
 * @file Ecat_KistFinger.h
 * @brief EtherCAT driver for KIST anthropomorphic hand finger motors (4-axis)
 * @date 2018-11-15
 * @author Junho Park
 *
 * v2.0 – O(1) direct PDO pointer access.
 *   PDO map (20 entries per KistHand_pdo_entries):
 *     [0..3]   0x7000 command 1-4      (RxPDO, S16 x4)
 *     [4..7]   0x6000 MotorPos 1-4     (TxPDO, S16 x4)
 *     [8..11]  0x6000 MotorVel 1-4     (TxPDO, S16 x4)
 *     [12..15] 0x6000 MotorInput 1-4   (TxPDO, S16 x4) — read only
 *     [16..17] 0x6000 MotorTorq 1-2    (TxPDO, S16 x2) — read only
 *     [18]     0x6000 Status            (TxPDO, S16)
 *     [19]     0x6000 Timer Tick        (TxPDO, U16)
 */

#pragma once

#include "ecat/core/Ecat_Slave.h"

//#define _KIST_FINGER_DEBUG_

namespace hyuEcat {

/**
 * @brief EtherCAT driver for KIST hand finger motor controller (4 motors)
 * @version 2.0.0
 */
class EcatKistFinger : public Slave
{
public:
    EcatKistFinger() : Slave(KistHand_VendorID, KistHand_ProductCode) {
        dc_enabled_         = true;
        dc_assign_activate_ = 0x0300;
        dc_sync0_shift_     = 400000;
    }
    ~EcatKistFinger() override = default;

    bool initialized() const noexcept { return initialized_; }
    int  GetDeviceState() const noexcept { return static_cast<int>(state_); }

    const char* GetDevstr() const noexcept
    {
        const int s = static_cast<int>(state_);
        return (s >= 0 && s < kNumStates) ? kStateStr[s] : "Unknown";
    }

    void SetVelocityTarget(const int16_t* velocity) noexcept
    {
        for (int i = 0; i < 4; ++i) command[i] = velocity[i];
    }

    // ================================================================
    //  RT interface (v2.0)
    // ================================================================

    /**
     * @brief Cache raw PDO pointers.  Called once by Master::activate().
     */
    void syncDomainMap(const std::vector<uint8_t*>& ptrs) override
    {
        if (ptrs.size() < 20) {
            fprintf(stderr, "WARNING. EcatKistFinger. syncDomainMap: expected 20 ptrs, got %zu\n",
                    ptrs.size());
            return;
        }
        // RxPDO (commands)
        for (int i = 0; i < 4; ++i) p_command_[i]    = ptrs[i];
        // TxPDO (motor data)
        for (int i = 0; i < 4; ++i) p_MotorPos_[i]   = ptrs[4 + i];
        for (int i = 0; i < 4; ++i) p_MotorVel_[i]   = ptrs[8 + i];
        for (int i = 0; i < 4; ++i) p_MotorInput_[i] = ptrs[12 + i];
        for (int i = 0; i < 2; ++i) p_MotorTorque_[i]= ptrs[16 + i];
        p_MotorStatus_ = ptrs[18];
        p_timer_tick_  = ptrs[19];
    }

    /**
     * @brief Read incoming TxPDO data (positions, velocities, torques, status).
     * Called once per cycle after ecrt_domain_process().
     */
    void updateRx() override
    {
        for (int i = 0; i < 4; ++i) MotorPos  [i] = EC_READ_S16(p_MotorPos_[i]);
        for (int i = 0; i < 4; ++i) MotorVel  [i] = EC_READ_S16(p_MotorVel_[i]);
        for (int i = 0; i < 4; ++i) MotorInput[i] = EC_READ_S16(p_MotorInput_[i]);
        for (int i = 0; i < 2; ++i) MotorTorque[i]= EC_READ_S16(p_MotorTorque_[i]);
        MotorStatus = EC_READ_S16(p_MotorStatus_);
        timer_tick  = EC_READ_U16(p_timer_tick_);
    }

    /**
     * @brief Write outgoing RxPDO data (velocity commands).
     * Called once per cycle before ecrt_domain_queue().
     */
    void updateTx() override
    {
        for (int i = 0; i < 4; ++i) EC_WRITE_S16(p_command_[i], command[i]);
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

    // ---- Cached PDO pointers (set by syncDomainMap) ----
    uint8_t* p_command_    [4] = {};
    uint8_t* p_MotorPos_   [4] = {};
    uint8_t* p_MotorVel_   [4] = {};
    uint8_t* p_MotorInput_ [4] = {};
    uint8_t* p_MotorTorque_[2] = {};
    uint8_t* p_MotorStatus_    = nullptr;
    uint8_t* p_timer_tick_     = nullptr;

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

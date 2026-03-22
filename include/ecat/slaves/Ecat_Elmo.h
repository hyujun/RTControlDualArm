/**
 * @file Ecat_Elmo.h
 * @date 2018-11-15
 * @author Junho Park
 *
 * v2.0 – O(1) direct PDO pointer access.
 *   PDO map (9 entries):
 *     [0] 0x6071 target torque         (RxPDO, S16)
 *     [1] 0x6040 controlword           (RxPDO, U16) — read-modify-write
 *     [2] 0x6072 max torque            (RxPDO, U16)
 *     [3] 0x6060 mode of operation     (RxPDO, S8)
 *     [4] 0x6069 velocity sensor value (TxPDO, S32)
 *     [5] 0x6064 position actual       (TxPDO, S32)
 *     [6] 0x6077 torque actual         (TxPDO, S16)
 *     [7] 0x6041 statusword            (TxPDO, U16)
 *     [8] 0x6061 modes of op display   (TxPDO, S8)
 */

#pragma once

#include "ecat/core/Ecat_Slave.h"

//#define _ELMO_DEBUG_

namespace hyuEcat {

/**
 * @brief EtherCAT driver for Elmo Gold-DC-Whistle (CiA 402, cyclic sync torque)
 * @version 2.0.0
 */
class EcatElmo : public Slave
{
public:
    EcatElmo() : Slave(Elmo_VendorID, Elmo_ProductCode) {}
    ~EcatElmo() override = default;

    bool initialized()      const noexcept { return initialized_; }
    bool isHoming()         const noexcept { return homing_; }
    int  Elmo_DeviceState() const noexcept { return static_cast<int>(state_); }

    void writeTorque  (int16_t torque)    noexcept { target_torque_   = torque;   }
    void writeVelocity(int32_t velocity)  noexcept { target_velocity_ = velocity; }
    void writePosition(int32_t position)  noexcept { target_position_ = position; }

    /**
     * @brief Returns a string literal for the current device state.
     * RT-safe: O(1) array lookup, no heap allocation, no copy.
     */
    const char* GetDevState() const noexcept
    {
        const int s = static_cast<int>(state_);
        return (s >= 0 && s < kNumStates) ? kStateStr[s] : "Unknown";
    }

    // ================================================================
    //  RT interface (v2.0)
    // ================================================================

    /**
     * @brief Cache raw PDO pointers.  Called once by Master::activate().
     * ptrs[i] = domain_pd + offset[i] for each channel i.
     */
    void syncDomainMap(const std::vector<uint8_t*>& ptrs) override
    {
        if (ptrs.size() < 9) {
            fprintf(stderr, "WARNING. EcatElmo. syncDomainMap: expected 9 ptrs, got %zu\n",
                    ptrs.size());
            return;
        }
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

    /**
     * @brief Read incoming TxPDO data (position, velocity, torque, status).
     * Called once per cycle after ecrt_domain_process().
     */
    void updateRx() override
    {
        velocity_ = EC_READ_S32(p_velocity_);
        position_ = EC_READ_S32(p_position_);
        torque_   = EC_READ_S16(p_torque_);

        const uint16_t sw = EC_READ_U16(p_statusword_);
        mode_of_operation_display_ = EC_READ_S8(p_mode_of_op_display_);

        if (sw != last_status_word_) {
            state_ = deviceState(sw);
#if defined(_ELMO_DEBUG_)
            fprintf(stdout, "ELMO pos:%d state:%s\n", slave_position, GetDevState());
#endif
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

    /**
     * @brief Write outgoing RxPDO data (torque, controlword, max_torque, mode).
     * Called once per cycle before ecrt_domain_queue().
     */
    void updateTx() override
    {
        EC_WRITE_S16(p_target_torque_, target_torque_);

        control_word_ = EC_READ_U16(p_controlword_);
        control_word_ = transition(state_, control_word_);
        EC_WRITE_U16(p_controlword_, control_word_);

        EC_WRITE_U16(p_max_torque_,        max_torque_);
        EC_WRITE_S8 (p_mode_of_operation_, mode_of_operation_);
    }

    // ---- PDO / sync descriptors ----
    const ec_sync_info_t*      syncs()    override { return Elmo_syncs; }
    size_t                     syncSize() override { return sizeof(Elmo_syncs) / sizeof(ec_sync_info_t); }
    const ec_pdo_entry_info_t* channels() override { return Elmo_pdo_entries; }
    void domains(DomainMap& d) const override { d = domains_; }

    // ---- PDO data (public for direct RT access) ----
    int32_t  target_position_           = 0;
    int32_t  target_velocity_           = 0;
    int16_t  target_torque_             = 0;
    uint16_t max_torque_                = 1300;
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

    // ---- Cached PDO pointers (set by syncDomainMap) ----
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

    // Static string table: O(1), zero heap allocation, shared across all instances.
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
};

// Out-of-class definition required in C++14 for static constexpr arrays
constexpr const char* EcatElmo::kStateStr[EcatElmo::kNumStates];

} /* namespace hyuEcat */

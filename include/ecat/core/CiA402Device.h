/**
 * @file CiA402Device.h
 * @brief Common Base Class for CiA 402 EtherCAT Motor Drives
 * @date 2026-03-27
 * @version 2.1.0
 */

#pragma once

#include <cstdint>
#include "ecat/core/Ecat_Slave.h"

namespace hyuEcat {

/**
 * @brief Generalized CiA 402 Device Base Class
 * Inherits from Slave and implements the standard CiA 402 state machine.
 */
class CiA402Device : public Slave
{
public:
    CiA402Device(uint32_t vendor_id, uint32_t product_id)
        : Slave(vendor_id, product_id) {}
    virtual ~CiA402Device() = default;

    bool initialized() const noexcept { return initialized_; }
    bool isHoming()    const noexcept { return homing_; }

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

    void writeTorque  (int16_t torque)   noexcept { target_torque_   = torque;   }
    void writeVelocity(int32_t velocity) noexcept { target_velocity_ = velocity; }
    void writePosition(int32_t position) noexcept { target_position_ = position; }
    
    void setModeOfOperation(ModeOfOperation mode) noexcept { mode_of_operation_ = mode; }

    int32_t position() const noexcept { return position_; }
    int32_t velocity() const noexcept { return velocity_; }
    int16_t torque()   const noexcept { return torque_;   }
    
    int  getDeviceStateAsInt() const noexcept { return static_cast<int>(state_); }

    const char* GetDevState() const noexcept
    {
        const int s = static_cast<int>(state_);
        return (s >= 0 && s < kNumStates) ? kStateStr[s] : "Unknown";
    }

protected:
    // ---- CiA 402 Device State Machine ----
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

    // ---- Status Tracking ----
    int         last_status_word_ = -1;
    DeviceState last_state_       = STATE_START;
    DeviceState state_            = STATE_START;

    bool initialized_  = false;
    bool homing_       = false;
    bool homing_ready_ = false;

    // ---- PDO data (common for CiA 402) ----
    int32_t  target_position_           = 0;
    int32_t  target_velocity_           = 0;
    int16_t  target_torque_             = 0;
    uint16_t max_torque_                = 1000;
    uint16_t control_word_              = 0;
    int8_t   mode_of_operation_         = MODE_CYCLIC_SYNC_TORQUE;

    int32_t  position_                  = 0;
    int32_t  velocity_                  = 0;
    int16_t  torque_                    = 0;
    uint16_t status_word_               = 0;
    int8_t   mode_of_operation_display_ = 0;
};

// Out-of-class definition required in C++14 for static constexpr arrays
constexpr const char* CiA402Device::kStateStr[CiA402Device::kNumStates];

} /* namespace hyuEcat */

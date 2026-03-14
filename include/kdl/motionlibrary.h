//
// Created by Junho Park on 2021/04/25.
//

#pragma once

#include <cstdint>

// Joint motion commands
inline constexpr uint8_t MOVE_ZERO        = 0x01;
inline constexpr uint8_t MOVE_JOB         = 0x02;
inline constexpr uint8_t MOVE_CUSTOMIZE1  = 0x03;
inline constexpr uint8_t MOVE_CUSTOMIZE2  = 0x04;
inline constexpr uint8_t MOVE_CUSTOMIZE3  = 0x05;
inline constexpr uint8_t MOVE_CUSTOMIZE4  = 0x06;
inline constexpr uint8_t MOVE_CUSTOMIZE5  = 0x07;  //drill preposition
inline constexpr uint8_t MOVE_CUSTOMIZE6  = 0x08;  //drill center
inline constexpr uint8_t MOVE_CUSTOMIZE7  = 0x09;  //drill1
inline constexpr uint8_t MOVE_CUSTOMIZE8  = 0x0a;  //drill1
inline constexpr uint8_t MOVE_CUSTOMIZE9  = 0x0b;  //drill1
inline constexpr uint8_t MOVE_CUSTOMIZE10 = 0x0c;  //drill2
inline constexpr uint8_t MOVE_CUSTOMIZE11 = 0x0d;  //drill2
inline constexpr uint8_t MOVE_CUSTOMIZE12 = 0x0e;  //drill2
inline constexpr uint8_t MOVE_CUSTOMIZE13 = 0x0f;  //drill3
inline constexpr uint8_t MOVE_CUSTOMIZE14 = 0x10;  //drill3
inline constexpr uint8_t MOVE_CUSTOMIZE15 = 0x11;  //drill3
inline constexpr uint8_t MOVE_CUSTOMIZE16 = 0x12;  //drill4
inline constexpr uint8_t MOVE_CUSTOMIZE17 = 0x13;  //drill4
inline constexpr uint8_t MOVE_CUSTOMIZE18 = 0x14;  //drill4
inline constexpr uint8_t MOVE_CUSTOMIZE19 = 0x15;  //demo1
inline constexpr uint8_t MOVE_CUSTOMIZE20 = 0x16;  //demo1

inline constexpr uint8_t MOVE_JOINT_CYCLIC = 0x06;
inline constexpr uint8_t MOVE_CUSTOMIZE    = 0x07;

// Motion state flags
inline constexpr uint8_t TARGET_MOVING    = 0x10;
inline constexpr uint8_t TARGET_ACHIEVED  = 0x50;
inline constexpr uint8_t TARGET_ZERO      = 0x20;
inline constexpr uint8_t TARGET_JOB       = 0x30;
inline constexpr uint8_t TARGET_CUSTOMIZE = 0x40;

inline constexpr uint8_t SYSTEM_BEGIN = 0xff;

// Control mode identifiers
inline constexpr uint8_t CTRLMODE_IDY_JOINT       = 0x01;
inline constexpr uint8_t CTRLMODE_IMPEDANCE_JOINT  = 0x02;
inline constexpr uint8_t CTRLMODE_CLIK             = 0x03;
inline constexpr uint8_t CTRLMODE_TASK             = 0x04;
inline constexpr uint8_t CTRLMODE_IMPEDANCE_TASK   = 0x05;

inline constexpr uint8_t CTRLMODE_FRICTIONID = 0x10;

// Task motion commands
inline constexpr uint8_t MOVE_TASK_CUSTOM   = 0x01;
inline constexpr uint8_t MOVE_TASK_CUSTOM1  = 0x02;
inline constexpr uint8_t MOVE_TASK_CUSTOM2  = 0x03;
inline constexpr uint8_t MOVE_TASK_CUSTOM3  = 0x04;
inline constexpr uint8_t MOVE_TASK_CUSTOM4  = 0x05;
inline constexpr uint8_t MOVE_TASK_CUSTOM5  = 0x06;
inline constexpr uint8_t MOVE_TASK_CUSTOM6  = 0x07;
inline constexpr uint8_t MOVE_TASK_CUSTOM7  = 0x08;
inline constexpr uint8_t MOVE_TASK_CUSTOM8  = 0x09;
inline constexpr uint8_t MOVE_TASK_CUSTOM9  = 0x0a;
inline constexpr uint8_t MOVE_TASK_CUSTOM10 = 0x0b;
inline constexpr uint8_t MOVE_TASK_CUSTOM11 = 0x0c;
inline constexpr uint8_t MOVE_TASK_CUSTOM12 = 0x0d;
inline constexpr uint8_t MOVE_TASK_CUSTOM13 = 0x0e;
inline constexpr uint8_t MOVE_TASK_CUSTOM14 = 0x0f;
inline constexpr uint8_t MOVE_TASK_CUSTOM15 = 0x10;

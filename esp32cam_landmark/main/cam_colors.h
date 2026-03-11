// cam_colors.h - RGB565 color thresholds for each landmark
//
// Camera must be set to AWB OFF, wb_mode=4 (home) before these work.
// R is 5-bit (0-31), G is 6-bit (0-63), B is 5-bit (0-31)
//
// To recalibrate: flash esp32cam_calibrate, point at the card, note R/G/B values.
// Set _MIN = reading - 3 and _MAX = reading + 3 as a starting point.
//
// Landmark IDs: 0=Red(off)  1=Orange  2=Pink  3=Yellow  4=Purple

#pragma once

// Red (LM:0) - disabled, reserved
#define RED_R_MIN   17
#define RED_R_MAX   28
#define RED_G_MIN   18
#define RED_G_MAX   31
#define RED_B_MIN    8
#define RED_B_MAX   16

// Orange (LM:1) - calibrated 2026-03-26, AWB off home WB
// B stays at 2-8 which cleanly separates it from pink (B >= 9)
#define ORN_R_MIN   23
#define ORN_R_MAX   31
#define ORN_G_MIN   30
#define ORN_G_MAX   44
#define ORN_B_MIN    2
#define ORN_B_MAX    8

// Pink (LM:2) - calibrated 2026-03-26, AWB off home WB
// B stays at 9-14 which separates it from orange (B <= 8)
#define PNK_R_MIN   23
#define PNK_R_MAX   31
#define PNK_G_MIN   22
#define PNK_G_MAX   37
#define PNK_B_MIN    9
#define PNK_B_MAX   14

// Yellow (LM:3) - not yet tested on robot
#define YLW_R_MIN   17
#define YLW_R_MAX   25
#define YLW_G_MIN   46
#define YLW_G_MAX   54
#define YLW_B_MIN   13
#define YLW_B_MAX   21

// Purple (LM:4) - center room, not yet tested on robot
#define PUR_R_MIN   13
#define PUR_R_MAX   21
#define PUR_G_MIN   27
#define PUR_G_MAX   35
#define PUR_B_MIN   18
#define PUR_B_MAX   26

// Fire/flame thresholds (used by FIRE detection, separate from landmarks)
// High R, medium G, very low B
#define FIRE_R_MIN   15
#define FIRE_R_MAX   31
#define FIRE_G_MIN   15
#define FIRE_G_MAX   39
#define FIRE_B_MIN    0
#define FIRE_B_MAX    8

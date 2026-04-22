// cam_colors.h - RGB565 thresholds for each landmark color
// AWB must be OFF (wb_mode=4)

#pragma once

// LM:1 Orange/Red combined - recalibrated 2026-04-20
// Both cards share R=24, B=8; G range spans red(26-32) and orange(33-38)
#define ORN_R_MIN   20
#define ORN_R_MAX   28
#define ORN_G_MIN   26
#define ORN_G_MAX   38
#define ORN_B_MIN    4
#define ORN_B_MAX   12

// LM:2 Green - recalibrated 2026-04-20
// R_MAX=15 separates from orange (R_MIN=22) and red (R_MIN=20)
#define GRN_R_MIN    7
#define GRN_R_MAX   15
#define GRN_G_MIN   28
#define GRN_G_MAX   36
#define GRN_B_MIN    2
#define GRN_B_MAX   10

// LM:3 Red - recalibrated 2026-04-20
// G_MAX=32 separates from orange (G_MIN=33); R_MIN=20 separates from green (R_MAX=15)
#define RD_R_MIN    20
#define RD_R_MAX    28
#define RD_G_MIN    26
#define RD_G_MAX    32
#define RD_B_MIN     5
#define RD_B_MAX    12

// Fire/flame thresholds
#define FIRE_R_MIN   15
#define FIRE_R_MAX   31
#define FIRE_G_MIN   15
#define FIRE_G_MAX   39
#define FIRE_B_MIN    0
#define FIRE_B_MAX    8

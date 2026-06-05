#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "main.h"

/* =========================================================
 * 电机使能电平
 * 你的原代码中 DM542 为低电平使能
 * ========================================================= */
#define MOTOR_EN_ACTIVE       GPIO_PIN_RESET
#define MOTOR_EN_DISABLE      GPIO_PIN_SET

/* =========================================================
 * 并联 SCARA 机械参数
 * ========================================================= */
#define RT_ACTIVE_LINK_MM     110.0f
#define RT_PASSIVE_LINK_MM    220.0f
#define RT_MOTOR_DISTANCE_MM  160.0f
#define RT_BASE_Y_MM          0.0f
#define RT_TWO_PI             6.2831853071795864769f

#endif

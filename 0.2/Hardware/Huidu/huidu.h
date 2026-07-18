/* ============================================================================
 * @file    huidu.h
 * @brief   灰度传感器驱动 - 0.2 灰度板 (8 路 MUX_ADC + 5/8 路 GPIO 软切换)
 *
 *   应用层用法 (主循环):
 *     1. main() 上电调 Huidu_Init(white[8], black[8]);  ← 标定 + 配引脚
 *     2. 主循环每帧 Huidu_Task();                         ← 采集 + 二值化
 *     3. UI 读 Huidu_GetDigital() / Huidu_GetAnalog(out[8]);
 *     4. 模式切换 (可选): Huidu_SetMode() 或 Huidu_NextMode();
 *
 * ============================================================================
 */
#ifndef __HUIDU_H__
#define __HUIDU_H__

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    HUIDU_MODE_MUX_ADC    = 0,    /* 默认: MUX + ADC, 8 路 */
    HUIDU_MODE_FIRST5_GPIO = 1,   /* GPIO 5 路 */
    HUIDU_MODE_ALL8_GPIO  = 2,    /* GPIO 8 路 */
} huidu_mode_t;

extern huidu_mode_t Huidu_Mode;

#define HUIDU_CH_COUNT   8

typedef struct {
    uint16_t Analog_value[8];
    uint16_t Normal_value[8];
    uint16_t Calibrated_white[8];
    uint16_t Calibrated_black[8];
    uint16_t Gray_white[8];
    uint16_t Gray_black[8];
    double   Normal_factor[8];
    double   bits;
    uint8_t  Digital;
    uint8_t  ok;
} Huidu_Sensor_t;

extern Huidu_Sensor_t g_huidu_sensor;

/* ============================================================================
 * API
 * ============================================================================ */

void Huidu_Init(const uint16_t white[8], const uint16_t black[8]);

/* 模式切换 (运行时动态切, 重配引脚方向) */
void     Huidu_SetMode(huidu_mode_t mode);
huidu_mode_t Huidu_GetMode(void);

/* 轮转到下一模式: MUX_ADC → FIRST5_GPIO → ALL8_GPIO → MUX_ADC (用户 04:06) */
void Huidu_NextMode(void);

/* 采集 + 处理 */
void Huidu_Task(void);

/* 读 API */
uint8_t Huidu_GetDigital(void);
void    Huidu_GetAnalog(uint16_t out[8]);
void    Huidu_GetNormal(uint16_t out[8]);

/* 调试: MUX 手动选通 */
void Huidu_SwitchAddress(uint8_t code);

#endif /* __HUIDU_H__ */
/* ============================================================================
 * @file    buzzer.c
 * @brief   蜂鸣器 (PA12, BEEP_A29) 简化控制 (移植自 v1.21/lightandbuzzer.c)
 *
 * ============================================================================
 * 调用方法 (主循环用法)
 * ============================================================================
 *
 *   上电 (main() 启动序列里调一次):
 *     Buzzer_Init();                     // 默认静音 (高电平)
 *
 *   while(1):
 *     Buzzer_On();                       // 响 (拉低)
 *     Buzzer_Off();                      // 静音 (拉高)
 *     Buzzer_Toggle();                   // 翻转
 *     if (Buzzer_IsOn())  { ... }        // 查当前状态: 1=响, 0=静音
 *
 * ============================================================================
 * 硬件: BEEP_A29_PIN = PA12 (syscfg 已配 OUTPUT)
 * 电平: 低电平响, 高电平静音 (有源蜂鸣器)
 * ============================================================================
 */
#include "ti_msp_dl_config.h"
#include "Hardware/Buzzer/buzzer.h"

static uint8_t s_buzzer_on = 0;

void Buzzer_Init(void) {
    /* SysConfig 已经把 BEEP 配成 OUTPUT, 默认低电平.
     * 这里强制设高 (静音), 同时记录状态. */
    DL_GPIO_setPins(BEEP_PORT, BEEP_A29_PIN);
    s_buzzer_on = 0;
}

void Buzzer_On(void) {
    DL_GPIO_clearPins(BEEP_PORT, BEEP_A29_PIN);   /* 低电平 → 响 */
    s_buzzer_on = 1;
}

void Buzzer_Off(void) {
    DL_GPIO_setPins(BEEP_PORT, BEEP_A29_PIN);     /* 高电平 → 静音 */
    s_buzzer_on = 0;
}

void Buzzer_Toggle(void) {
    if (s_buzzer_on) {
        Buzzer_Off();
    } else {
        Buzzer_On();
    }
}

uint8_t Buzzer_IsOn(void) {
    return s_buzzer_on;
}
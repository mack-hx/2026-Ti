#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include "Hardware/MPU9250_Hardware_iic/mpu9250.h"
#include "Hardware/PD42S1/pd42s1.h"
#include "Hardware/PD42S1/stepmotor.h"
#include "LCD.h"
#include "system/clock.h"

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();

    LCD_Init(BLUE);
    NVIC_EnableIRQ(MPU9250_TIM_INST_INT_IRQN);
    MPU9250_Init();

    /* 步进电机初始化 */
    PD42S1_Init(PD42S1_BAUD_RATE);
    NVIC_EnableIRQ(x_bujin_INST_INT_IRQN);
    NVIC_EnableIRQ(y_bujin_INST_INT_IRQN);
    SM_Init();
    mspm0_delay_ms(3000);
    SM_MoveTo(SM_X, R, 20, 15, 26000);
    SM_Tick();
    SM_MoveTo(SM_Y, R, 20, 15, 14222);
    SM_Tick();

    while (1)
    {
        SM_Tick();
        MPU9250_Task();
        const MPU9250_Data *d = MPU9250_GetData();

        /* 标题 + 状态 */
        LCD_Printf(0, 0, WHITE, BLUE, LCD_6X12, LCD_modeoff,
                   "MPU9250 %s", d->ok ? "OK" : "--");

        /* 加速度 (三轴一行) */
        LCD_Printf(0, 14, GREEN, BLUE, LCD_6X12, LCD_modeoff,
                   "A %+5.2f %+5.2f %+5.2f",
                   (double)d->ax, (double)d->ay, (double)d->az);

        /* 陀螺仪 (三轴一行) */
        LCD_Printf(0, 28, GREEN, BLUE, LCD_6X12, LCD_modeoff,
                   "G %+5.1f %+5.1f %+5.1f",
                   (double)d->gx, (double)d->gy, (double)d->gz);

        /* 磁力计 (三轴一行) */
        LCD_Printf(0, 42, GREEN, BLUE, LCD_6X12, LCD_modeoff,
                   "M %+5.1f %+5.1f %+5.1f",
                   (double)d->mx, (double)d->my, (double)d->mz);

        /* 姿态角 */
        LCD_Printf(0, 60, YELLOW, BLUE, LCD_6X12, LCD_modeoff,
                   "Y%5.1f P%5.1f R%5.1f",
                   (double)d->yaw, (double)d->pitch, (double)d->roll);

        /* 温度 + 校准 */
        LCD_Printf(0, 74, WHITE, BLUE, LCD_6X12, LCD_modeoff,
                   "T%4.1f CAL:%s",
                   (double)d->temp_c,
                   d->calib_state == 2 ? "OK" : "...");
    }
}

// X轴 UART中断
void x_bujin_INST_IRQHandler(void)
{
    if (DL_UART_getPendingInterrupt(x_bujin_INST) == DL_UART_IIDX_RX)
    {
        PD42S1_UART_CallbackFor(0x01, DL_UART_receiveData(x_bujin_INST));
    }
}

// Y轴 UART中断
void y_bujin_INST_IRQHandler(void)
{
    if (DL_UART_getPendingInterrupt(y_bujin_INST) == DL_UART_IIDX_RX)
    {
        PD42S1_UART_CallbackFor(0x02, DL_UART_receiveData(y_bujin_INST));
    }
}

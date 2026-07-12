#include "ti_msp_dl_config.h"
#include "Hardware/PD42S1/stepmotor.h"
#include "Hardware/PD42S1/pd42s1.h"
#include "Hardware/KEY/key.h"
#include "Hardware/LED/led.h"
#include "user/UI/ui.h"
#include "LCD.h"
#include "system/clock.h"

#define RUN_RPM         60
#define RUN_ACCEL       100
#define HOME_RPM        100
#define HOME_LIMIT_MA   10

int main(void) {
    SYSCFG_DL_init();
    SysTick_Init();
    NVIC_EnableIRQ(x_bujin_INST_INT_IRQN);
    NVIC_EnableIRQ(y_bujin_INST_INT_IRQN);

    LED_Init();
    KEY_Init();
    PD42S1_Init(PD42S1_BAUD_RATE);
    SM_Init();
    LCD_Init(BLUE);

    while (1) {
        if (key(1, down) && !key_pressed(2))     
            SM_Run(SM_X, R, RUN_ACCEL, RUN_RPM);
        else if (key(2, down) && !key_pressed(1))
            SM_Run(SM_X, L, RUN_ACCEL, RUN_RPM);
        else if (key(1, up) || key(2, up))       
            SM_Stop(SM_X);
        else if (key(3, down))                 
            SM_ReadPosition(SM_X);
        else if (key(4, down))
            SM_zero(SM_X, HM);
        else if (key(5, down))
            SM_zeroset(SM_X, OL, 10000, true);

        SM_Tick();
        UI_Render();
    }
}

/* ============================================================================
 * UART 中断
 * ============================================================================ */
void x_bujin_INST_IRQHandler(void) {
    if (DL_UART_getPendingInterrupt(x_bujin_INST) == DL_UART_IIDX_RX) {
        PD42S1_UART_Callback(DL_UART_receiveData(x_bujin_INST));
    }
}

void y_bujin_INST_IRQHandler(void) {
    if (DL_UART_getPendingInterrupt(y_bujin_INST) == DL_UART_IIDX_RX) {
        (void)DL_UART_receiveData(y_bujin_INST);
    }
}
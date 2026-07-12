/**
 * @file    ui.c
 * @brief   128x160 竖屏 LCD 渲染 (无标题, 紧凑 6 行布局)
 *
 *   y=   0 ───── 键位 K1~K5: "k:1 1 1 1 1"  (按下哪个哪个变 0)
 *   y=  18 ───── POS + ERR + STATE
 *   y=  36 ───── TX: [b0..b6]   (HEAD ADDR FUNC DIR ACCEL SPD_HI SPD_LO, 7 字节满行)
 *   y=  54 ───── TX: [b7..n-1]  (剩余字节: pulses + chk + tail, 12字节帧显示 5 字节)
 *   y=  72 ───── RX: [HEAD ADDR FUNC ERR]
 *   y=  90 ───── RX: [data[1]..data[4]]  (读位置应答的 4 字节位置)
 *
 *   屏幕下方 (y >= 106) 留空, 不再画东西。
 */
#include "user/UI/ui.h"
#include "Hardware/PD42S1/stepmotor.h"
#include "Hardware/PD42S1/pd42s1.h"
#include "Hardware/KEY/key.h"
#include "LCD.h"
#include <stdio.h>

/* 每个文本行的 y 起点 (行高 16 px, 行间 2 px, 共 18 px/行) */
#define ROW_Y(n)  ((n) * 18)

void UI_Render(void) {
    char line[24];

    /* 行 1 (y=0): 5 个键位状态 (0=按下, 1=松开) */
    LCD_Fill(0, ROW_Y(0), LCD_W, ROW_Y(0) + 16, BLACK);
    snprintf(line, sizeof(line), "k:%d %d %d %d %d",
             key_pressed(1) ? 0 : 1,
             key_pressed(2) ? 0 : 1,
             key_pressed(3) ? 0 : 1,
             key_pressed(4) ? 0 : 1,
             key_pressed(5) ? 0 : 1);
    LCD_ShowString(0, ROW_Y(0), line, WHITE, BLACK, LCD_8X16, 0);

    /* 行 2 (y=18): POS + ERR + STATE */
    const char *state_str =
        (sm_state == 1) ? "FWD"  :
        (sm_state == 2) ? "REV"  :
        (sm_state == 3) ? "POS"  : "IDLE";
    LCD_Fill(0, ROW_Y(1), LCD_W, ROW_Y(1) + 16, BLACK);
    snprintf(line, sizeof(line), "P:%4ld", (long)sm_pos);
    LCD_ShowString(0,  ROW_Y(1), line, GREEN, BLACK, LCD_8X16, 0);
    snprintf(line, sizeof(line), "E:%02X", sm_err);
    LCD_ShowString(56, ROW_Y(1), line, sm_err == 0x01 ? GREEN : RED, BLACK, LCD_8X16, 0);
    LCD_ShowString(96, ROW_Y(1), (char *)state_str, YELLOW, BLACK, LCD_8X16, 0);

    /* 行 3 (y=36): TX 第 1 行 [b0..b6]  (7 字节, HEAD..pulses 高字节)
     * 行 4 (y=54): TX 第 2 行 [b7..b12] (剩字节, 12字节帧显示 5 字节 = pulses + chk + tail;
     *             LCD 14 字符宽 = 7 字节, 留余量防越界) */
    LCD_Fill(0, ROW_Y(2), LCD_W, ROW_Y(2) + 16, BLACK);
    LCD_Fill(0, ROW_Y(3), LCD_W, ROW_Y(3) + 16, BLACK);
    LCD_ShowString(0, ROW_Y(2), "T:", YELLOW, BLACK, LCD_8X16, 0);
    if (g_tx_fired && g_tx_buffer_len >= 4) {
        uint8_t *b = (uint8_t *)g_tx_buffer;
        uint8_t n = g_tx_buffer_len;
        /* 第 1 行: 前 7 字节 (b[0..6]) — 14 字符, 满行 */
        snprintf(line, sizeof(line), "%02X%02X%02X%02X%02X%02X%02X",
                 b[0], b[1], b[2], b[3], b[4], b[5], b[6]);
        LCD_ShowString(16, ROW_Y(2), line, CYAN, BLACK, LCD_8X16, 0);
        /* 第 2 行: 剩余字节 (b[7..n-1]) — 最多 7 字节, 不超 line[24] */
        uint8_t r = (n > 7) ? (n - 7) : 0;
        if (r > 7) r = 7;
        if (r > 0) {
            char *p = line;
            for (uint8_t i = 0; i < r; i++) {
                p += snprintf(p, 3, "%02X", b[7 + i]);
            }
            *p = '\0';
            LCD_ShowString(16, ROW_Y(3), line, CYAN, BLACK, LCD_8X16, 0);
        }
    } else {
        LCD_ShowString(16, ROW_Y(2), "-----------", GRAY, BLACK, LCD_8X16, 0);
    }

    /* 行 5 (y=72): RX 第 1 行 [HEAD ADDR FUNC ERR]
     * 行 6 (y=90): RX 第 2 行 [data[1..4]] (读位置应答的位置四字节) */
    LCD_Fill(0, ROW_Y(4), LCD_W, ROW_Y(4) + 16, BLACK);
    LCD_Fill(0, ROW_Y(5), LCD_W, ROW_Y(5) + 16, BLACK);
    LCD_ShowString(0, ROW_Y(4), "R:", YELLOW, BLACK, LCD_8X16, 0);
    pd42_frame_t *f = PD42S1_GetFrame();

    /* 0x2A 读位置应答: data[0]=ERR, data[1..4]=位置 (int32 大端), data_len=5 */
    if (f->function_code == PD42_FCT_READ_POSITION && f->data_len >= 5) {
        sm_pos = (int32_t)((uint32_t)f->data[1] << 24)
               | ((uint32_t)f->data[2] << 16)
               | ((uint32_t)f->data[3] << 8)
               |  (uint32_t)f->data[4];
        sm_err = f->data[0];
        SM_AckFrame(f->function_code);   /* 通知节流器: 收到应答, 停止重试 */
    }

    if (f->data_len > 0 || f->function_code != 0) {
        snprintf(line, sizeof(line), "%02X%02X%02X%02X",
                 PD42S1_FRAME_HEAD, f->slave_addr, f->function_code, f->data[0]);
        LCD_ShowString(16, ROW_Y(4), line, GREEN, BLACK, LCD_8X16, 0);
        if (f->data_len > 4) {
            snprintf(line, sizeof(line), "%02X%02X%02X%02X",
                     f->data[1], f->data[2], f->data[3], f->data[4]);
            LCD_ShowString(16, ROW_Y(5), line, GREEN, BLACK, LCD_8X16, 0);
        }
    } else {
        LCD_ShowString(16, ROW_Y(4), "----", GRAY, BLACK, LCD_8X16, 0);
    }
}

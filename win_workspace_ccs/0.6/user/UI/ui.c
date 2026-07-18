/* ============================================================================
 * ui.c - LCD 多页面 UI (128x160, 8x16 字体, 16px 行高, 10 行)
 *
 * 架构:
 *   - g_page == 0      → 菜单页
 *   - g_page == 1..10  → 详情页 (主页 / Task1-5 / Motor-X/Y / Huitb / MPU9250)
 *   - pages[]          → 注册表 (render / on_key / footer_hook)
 *   - 行级脏位渲染     → row_put / row_flush / mark_all_dirty
 *
 * 调用:
 *   UI_Init()       → 上电一次
 *   UI_Render()     → 每帧
 *   UI_ForceRedraw()→ 切页 / 按键
 *
 * 颜色约定: WHITE=普通 YELLOW=选中 GREEN=成功 RED=错误 GRAY=占位 CYAN=调试
 * ============================================================================
 */
#include "user/UI/ui.h"
#include "Hardware/PD42S1/stepmotor.h"
#include "Hardware/PD42S1/pd42s1.h"
#include "Hardware/KEY/key.h"
#include "Hardware/LED/led.h"
#include "Hardware/Huidu/huidu.h"
#include "Hardware/TB6612/tb6612.h"
#include "Hardware/Encoder/encoder.h"
#include "Hardware/MPU9250/mpu9250.h"
#include "Hardware/Buzzer/buzzer.h"
#include "Hardware/UART_Host/uart_host.h"
#include "LCD.h"
#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <string.h>

/* ============================================================================
 * 顶部参数表 - 改这里调参，不碰底层 API
 * ============================================================================ */

/* 移动精度档 (K4 短按循环, 单位 0.1°) */
static const uint16_t kStepTenths[] = {10, 30, 50, 100, 300, 600, 900, 1};
/* 移动速度档 (长按 K2 循环, RPM) */
static const uint16_t kSpeedRpm[]   = {7, 10, 15, 25, 40};
/* 移动加速度档 (长按 K1 循环, 0~200) */
static const uint8_t  kAccel[]      = {10, 15, 20, 30, 50};

/* 上电默认: 进主页 (g_page=1), 菜单选中 TASK1 (下标 2) */
#ifndef MAIN_DEFAULT_PAGE
#define MAIN_DEFAULT_PAGE  1U
#endif
#define MENU_BOOT_SEL      2U
#define STEP_DEFAULT_IDX   3U   // 10°/次
#define SPEED_DEFAULT_IDX  2U   // 15 RPM
#define ACCEL_DEFAULT_IDX  2U   // 20

/* 步进电机零点参数 */
#define PULSES_PER_REV   51200L
#define X_LEFT_DEG       0
#define X_RIGHT_DEG      1800
#define Y_LEFT_DEG       0
#define Y_RIGHT_DEG      220
#define ZEROSET_TIMEOUT  10000U

// 度 → 脉冲 (四舍五入)
#define DEG_TO_PULSE(deg) \
    ((int32_t)(((int64_t)(deg) * PULSES_PER_REV + 180LL) / 360LL))

/* ============================================================================
 * 全局状态
 * ============================================================================ */
uint8_t       g_page     = MAIN_DEFAULT_PAGE;
uint8_t       g_menu_sel = MENU_BOOT_SEL;
const uint8_t PAGE_COUNT = 11;

/* X/Y 轴独立档位状态 */
static uint8_t s_step_idx[2]  = {STEP_DEFAULT_IDX, STEP_DEFAULT_IDX};
static uint8_t s_speed_idx[2] = {SPEED_DEFAULT_IDX, SPEED_DEFAULT_IDX};
static uint8_t s_accel_idx[2] = {ACCEL_DEFAULT_IDX, ACCEL_DEFAULT_IDX};

/* 主页 K1 toggle TB6612 状态 */
static bool s_main_tb_running = false;

/* ============================================================================
 * ==================== 行渲染框架 ====================
 * ============================================================================ */
#define LCD_ROWS  10U

typedef struct {
    char     text[LCD_ROW_CHARS + 2];
    uint16_t hash;
    uint16_t fg, bg;
    uint8_t  dirty;
} row_buf_t;

static row_buf_t s_rows[LCD_ROWS];
static uint8_t   s_row_count = 0;

// FNV-1a 16-bit 哈希
static uint16_t fnv1a16(const char *s) {
    uint16_t h = 21661u;
    while (*s) h = (uint16_t)((h ^ (uint8_t)*s++) * 16777u);
    return h;
}

// 写一行 (内容不变时跳过 LCD 写入)
static void row_put(uint8_t idx, const char *txt, uint16_t fg, uint16_t bg) {
    if (idx >= LCD_ROWS) return;

    char buf[LCD_ROW_CHARS + 2];
    uint8_t i = 0;
    while (i < LCD_ROW_CHARS && txt[i]) {
        buf[i] = txt[i];
        i++;
    }
    buf[i] = '\0';

    uint16_t h = fnv1a16(buf);
    row_buf_t *r = &s_rows[idx];
    if (!r->dirty && r->hash == h && strcmp(r->text, buf) == 0) {
        if (idx + 1 > s_row_count) s_row_count = idx + 1;
        return;
    }
    memcpy(r->text, buf, i + 1);
    r->hash = h; r->fg = fg; r->bg = bg; r->dirty = 1;
    if (idx + 1 > s_row_count) s_row_count = idx + 1;
}

// 刷 LCD (只写 dirty 行)
static void row_flush(void) {
    for (uint8_t i = 0; i < s_row_count; i++) {
        row_buf_t *r = &s_rows[i];
        if (!r->dirty) continue;
        LCD_Fill(0, i * 16, LCD_W, (i + 1) * 16, r->bg);
        LCD_ShowString(0, i * 16, r->text, r->fg, r->bg, LCD_8X16, 0);
        r->dirty = 0;
    }
}

// 切页时强制全屏重画
static void mark_all_dirty(void) {
    for (uint8_t i = 0; i < LCD_ROWS; i++) {
        s_rows[i].dirty = 1;
        s_rows[i].text[0] = '\0';
        s_rows[i].hash = 0;
    }
    s_row_count = 0;
}

// ROW 0 键位条: "k:1 1 1 1 1" (1=松开, 0=按下)
static void row_put_keys(void) {
    char line[LCD_ROW_CHARS + 2];
    snprintf(line, sizeof(line), "k:%u %u %u %u %u",
             key_pressed(1) ? 1U : 0U, key_pressed(2) ? 1U : 0U,
             key_pressed(3) ? 1U : 0U, key_pressed(4) ? 1U : 0U,
             key_pressed(5) ? 1U : 0U);
    row_put(0, line, WHITE, BLACK);
}

/* ============================================================================
 * ==================== 菜单页 MenuPage ====================
//
//  ROW 0: "   == MENU ==   " 标题
//  ROW 1..8: 8 个菜单项 (选中项前缀 '>' + YELLOW)
//  ROW 9: "P1/11 MENU" 页脚
//
//  K1/K2/K5 路由在 main.c, on_key 是空函数
 * ============================================================================ */
static const char * const kMenuNames[MENU_ITEM_COUNT] = {
    "MAIN", "TASK1", "TASK2", "TASK3", "TASK4", "TASK5",
    "MOTOR-X", "MOTOR-Y", "HUITB", "MPU9250",
};

static void MenuPage_Render(void) {
    row_put(0, "   == MENU ==   ", WHITE, DARKBLUE);
    char line[LCD_ROW_CHARS + 2];

    // 9 项菜单只有 8 行, 选中 ≥6 时窗口下移
    uint8_t first = (g_menu_sel > 8) ? (uint8_t)(g_menu_sel - 7) : 1;
    for (uint8_t row = 0; row < 8; row++) {
        uint8_t sel = first + row;
        snprintf(line, sizeof(line), "%c%-15.15s",
                 (g_menu_sel == sel) ? '>' : ' ', kMenuNames[sel - 1]);
        row_put(1 + row, line, (g_menu_sel == sel) ? YELLOW : WHITE, BLACK);
    }
}

static void MenuPage_OnKey(void) { /* 空函数, 路由在 main.c */ }

/* ============================================================================
 * ==================== 主页 MainPage ====================
//
//  ROW 0: k:1 1 1 1 1      键位
//  ROW 1: 灰度二值化          8/5 路
//  ROW 2: X:+12345Y:-6789    X/Y 轴位置 (错开读)
//  ROW 3: SPX:XXRPM AC:XXX   X 轴速度+加速度
//  ROW 4: SPY:XXRPM AC:XXX   Y 轴速度+加速度
//  ROW 5: L:+12345R:-6789    编码器 L/R
//  ROW 6: PWM:>Axxx Bxxx     TB6612 两路 PWM
//  ROW 7: +1.00+2.00-0.50    陀螺三轴
//  ROW 8: K230:+123,+1280     K230 坐标
//  ROW 9: P2/11 main         页脚
//
//  按键:
//    K1: toggle TB6612 两电机 FWD
//    K2: 激光开/关
 * ============================================================================ */
static void MainPage_Render(void) {
    char line[LCD_ROW_CHARS + 2];

    // ROW 0 键位
    snprintf(line, sizeof(line), "k:%u %u %u %u %u",
             key_pressed(1) ? 1U : 0U, key_pressed(2) ? 1U : 0U,
             key_pressed(3) ? 1U : 0U, key_pressed(4) ? 1U : 0U,
             key_pressed(5) ? 1U : 0U);
    row_put(0, line, WHITE, BLACK);

    // ROW 1 灰度二值化 (按模式画 5/8 位)
    {
        uint8_t digital = Huidu_GetDigital();
        huidu_mode_t mode = Huidu_GetMode();
        char *p = line;
        uint8_t cnt = (mode == HUIDU_MODE_FIRST5_GPIO) ? 5 : 8;
        for (uint8_t i = 0; i < cnt; i++) {
            *p++ = (digital & (1 << i)) ? '1' : '0';
            if (i < cnt - 1) *p++ = ' ';
        }
        *p = '\0';
        row_put(1, line, WHITE, BLACK);
    }

    // ROW 2 X/Y 轴位置
    snprintf(line, sizeof(line), "X:%+6ldY:%+6ld",
             (long)SM_GetPosition(SM_X), (long)SM_GetPosition(SM_Y));
    row_put(2, line, WHITE, BLACK);

    // ROW 3/4 X/Y 轴速度+加速度
    for (uint8_t i = 0; i < 2; i++) {
        uint8_t spd = kSpeedRpm[s_speed_idx[i] < 5 ? s_speed_idx[i] : 2];
        uint8_t acc = kAccel[s_accel_idx[i] < 5 ? s_accel_idx[i] : 2];
        snprintf(line, sizeof(line), "SP%c:%2uRPM AC:%3u",
                 i ? 'Y' : 'X', (unsigned)spd, (unsigned)acc);
        row_put(3 + i, line, YELLOW, BLACK);
    }

    // ROW 5 编码器
    snprintf(line, sizeof(line), "L:%+6ldR:%+6ld",
             (long)Encoder_GetCountL(), (long)Encoder_GetCountR());
    row_put(5, line, WHITE, BLACK);

    // ROW 6 TB6612 PWM (running 时加 '>' 前缀)
    {
        uint16_t ca = TB6612_GetCompare(TB_MOTOR_L);
        uint16_t cb = TB6612_GetCompare(TB_MOTOR_R);
        snprintf(line, sizeof(line), "PWM:%cA%3uB%3u",
                 s_main_tb_running ? '>' : ' ', (unsigned)ca / 20, (unsigned)cb / 20);
        row_put(6, line, s_main_tb_running ? GREEN : WHITE, BLACK);
    }

    // ROW 7 陀螺三轴
    {
        const MPU9250_Data *d = MPU9250_GetData();
        if (d->ok) {
            snprintf(line, sizeof(line), "%+5.2f%+5.2f%+5.2f",
                     (double)d->gx, (double)d->gy, (double)d->gz);
            row_put(7, line, WHITE, BLACK);
        } else {
            row_put(7, "G: no dev       ", GRAY, BLACK);
        }
    }

    // ROW 8 K230 坐标
    if (g_k230_cmd.has_coord) {
        snprintf(line, sizeof(line), "K230:%+5ld,%+5ld",
                 (long)g_k230_cmd.coord_x, (long)g_k230_cmd.coord_y);
        row_put(8, line, GREEN, BLACK);
    } else if (!g_k230_cmd.recognition_ok) {
        row_put(8, "K230:no rec      ", RED, BLACK);
    } else {
        row_put(8, "K230:no link     ", GRAY, BLACK);
    }
}

static void MainPage_OnKey(void) {
    // K1: toggle TB6612 两电机 FWD
    if (key(1, down)) {
        if (s_main_tb_running) {
            TB6612_Stop(TB_MOTOR_L);
            TB6612_Stop(TB_MOTOR_R);
            s_main_tb_running = false;
        } else {
            TB6612_Run(TB_MOTOR_L, TB_DIR_FWD);
            TB6612_Run(TB_MOTOR_R, TB_DIR_FWD);
            s_main_tb_running = true;
        }
        UI_ForceRedraw();
    }

    // K2: 激光开/关
    if (key(2, down)) { UART_SendString(UART_CH1, "激光打开\r\n"); UI_ForceRedraw(); }
    if (key(2, up))   { UART_SendString(UART_CH1, "激光关闭\r\n"); UI_ForceRedraw(); }
}

/* ============================================================================
 * ==================== Task 页 (占位) ====================
 * ============================================================================ */
static void TaskPage_Render(uint8_t n) {
    row_put_keys();
    char line[LCD_ROW_CHARS + 2];
    snprintf(line, sizeof(line), "     TASK%u     ", (unsigned)n);
    row_put(1, line, YELLOW, BLACK);
    row_put(2, "  to be added   ", GRAY, BLACK);
    for (uint8_t i = 3; i <= 8; i++) row_put(i, "", GRAY, BLACK);
}
static void TaskPage_OnKey(void) {}
static void Task1Page_Render(void) { TaskPage_Render(1); }
static void Task2Page_Render(void) { TaskPage_Render(2); }
static void Task3Page_Render(void) { TaskPage_Render(3); }
static void Task4Page_Render(void) { TaskPage_Render(4); }
static void Task5Page_Render(void) { TaskPage_Render(5); }

/* ============================================================================
 * ==================== 电机页 MotorPage (X/Y 共用) ====================
//
//  ROW 0: k:1 1 1 1 1    键位
//  ROW 1: AX:X STEP:10.0  轴+步长档
//  ROW 2: POS:+12345       当前位置 [脉冲]
//  ROW 3: DEG:+123.45      位置 [度]
//  ROW 4: T:XX XX XX...    TX 原始 hex
//  ROW 5: T:<续>
//  ROW 6: R:XX XX XX...    RX 原始 hex
//  ROW 7: R:<续>
//  ROW 8: S:XXRPM A:XXX   当前速度+加速度档
//  ROW 9: P7/11 MOTOR-X   页脚
//
//  按键:
//    K1 按下: 正转 | 松开: 停
//    K2 按下: 反转 | 松开: 停
//    K3 短按: 读位置 | 长按: 触发回零
//    K4 短按: 切精度档 | 长按: 写入 zeroset 6 帧
//    K1 长按: 切加速度档
//    K2 长按: 切速度档
 * ============================================================================ */

// 读取当前档位 (简写函数)
static uint8_t M_Ax(sm_motor_t m) { return m == SM_Y ? 1 : 0; }
static int32_t M_Step(sm_motor_t m) {  // 当前步进脉冲数
    uint8_t ax = M_Ax(m);
    uint16_t t = kStepTenths[s_step_idx[ax]];
    return (int32_t)((uint32_t)t * PULSES_PER_REV / 3600);
}
static uint16_t M_Spd(sm_motor_t m) {  // 当前速度 RPM
    uint8_t ax = M_Ax(m), i = s_speed_idx[ax];
    return kSpeedRpm[i < 5 ? i : 0];
}
static uint8_t M_Acc(sm_motor_t m) {   // 当前加速度
    uint8_t ax = M_Ax(m), i = s_accel_idx[ax];
    return kAccel[i < 5 ? i : 0];
}

// 写入 zeroset 6 帧
static void MotorZeroset(sm_motor_t m) {
    if (m == SM_X) {
        SM_zeroset(SM_X, DEG_TO_PULSE(X_LEFT_DEG), DEG_TO_PULSE(X_RIGHT_DEG),
                   ZEROSET_TIMEOUT, true, true);
    } else {
        SM_zeroset(SM_Y, DEG_TO_PULSE(Y_LEFT_DEG), DEG_TO_PULSE(Y_RIGHT_DEG),
                   ZEROSET_TIMEOUT, true, true);
    }
}

// TX/RX hex 兜底 (拆两行)
static void MotorPutHex(uint8_t row, char prefix, const volatile uint8_t *d, uint8_t len, uint16_t color) {
    char line[LCD_ROW_CHARS + 2];
    if (d == NULL || len == 0) {
        snprintf(line, sizeof(line), "%c:------------", prefix);
        row_put(row, line, GRAY, BLACK);
        row_put(row + 1, "", GRAY, BLACK);
        return;
    }
    uint8_t n = len > 7 ? 7 : len;
    snprintf(line, sizeof(line), "%c:", prefix);
    char *p = line + 2;
    for (uint8_t i = 0; i < n; i++) p += snprintf(p, 3, "%02X", (unsigned)d[i]);
    row_put(row, line, color, BLACK);

    uint8_t r = len > 7 ? (len - 7) : 0;
    if (r > 8) r = 8;
    p = line;
    for (uint8_t i = 0; i < r; i++) p += snprintf(p, 3, "%02X", (unsigned)d[7 + i]);
    row_put(row + 1, line, color, BLACK);
}

// 帧 → 原始字节
static uint8_t FrameToRaw(pd42_frame_t *f, uint8_t *out) {
    uint8_t n = 0;
    out[n++] = PD42S1_FRAME_HEAD;
    out[n++] = f->slave_addr;
    out[n++] = f->function_code;
    for (uint8_t i = 0; i < f->data_len && n < 8; i++) out[n++] = f->data[i];
    out[n++] = f->checksum;
    out[n++] = PD42S1_FRAME_TAIL;
    return n;
}

static void MotorPage_Render(sm_motor_t m) {
    char line[LCD_ROW_CHARS + 2];
    uint8_t ax = M_Ax(m);

    row_put_keys();

    // ROW 1 轴 + 步长
    snprintf(line, sizeof(line), "AX:%c STEP:%4.1f",
             m == SM_X ? 'X' : 'Y', (double)kStepTenths[s_step_idx[ax]] / 10.0);
    row_put(1, line, YELLOW, BLACK);

    // ROW 2/3 位置
    int32_t pos = SM_GetPosition(m);
    double deg = (double)pos * 360.0 / PULSES_PER_REV;
    snprintf(line, sizeof(line), "POS:%+11ld", (long)pos);
    row_put(2, line, WHITE, BLACK);
    snprintf(line, sizeof(line), "DEG:%+12.2f", deg);
    row_put(3, line, GREEN, BLACK);

    // ROW 4/5 TX hex
    uint8_t tx_len = 0;
    bool tx_fired = false;
    const volatile uint8_t *tx = PD42S1_GetTxBuffer((uint8_t)m, &tx_len, &tx_fired);
    MotorPutHex(4, 'T', tx, tx_fired ? tx_len : 0, CYAN);

    // ROW 6/7 RX hex
    pd42_frame_t *rx = PD42S1_GetFrameFor((uint8_t)m);
    uint8_t raw[10], raw_len = 0;
    if (rx->function_code) raw_len = FrameToRaw(rx, raw);
    uint16_t rx_color = (raw_len && rx->error_code == PD42_ACK_OK) ? GREEN : RED;
    MotorPutHex(6, 'R', raw, raw_len, rx_color);

    // ROW 8 速度+加速度档
    snprintf(line, sizeof(line), "S:%3uRPM A:%3u", (unsigned)M_Spd(m), (unsigned)M_Acc(m));
    row_put(8, line, YELLOW, BLACK);
}

static void MotorPage_OnKey(sm_motor_t m) {
    // K1 按下: 正转 | 松开: 停 + 读位置
    if (key(1, down)) {
        SM_Move(m, R, M_Acc(m), M_Spd(m), M_Step(m));
        UI_ForceRedraw();
    }
    if (key(1, up)) {
        SM_Stop(m);
        SM_ReadPosition(m);
    }

    // K2 按下: 反转 | 松开: 停 + 读位置
    if (key(2, down)) {
        SM_Move(m, L, M_Acc(m), M_Spd(m), M_Step(m));
        UI_ForceRedraw();
    }
    if (key(2, up)) {
        SM_Stop(m);
        SM_ReadPosition(m);
    }

    // K3 短按: 读位置 | 长按: 触发回零
    if (key(3, down)) { SM_ReadPosition(m); UI_ForceRedraw(); }
    if (key(3, long_press)) { SM_zero(m, HM); UI_ForceRedraw(); }

    // K4 短按: 切精度档 (松开时切, 避免长按误触)
    if (key(4, up)) {
        s_step_idx[M_Ax(m)] = (s_step_idx[M_Ax(m)] + 1) % 8;
        UI_ForceRedraw();
    }

    // K4 长按: 写入 zeroset 6 帧
    if (key(4, long_press)) { MotorZeroset(m); UI_ForceRedraw(); }

    // K1 长按: 切加速度档 | K2 长按: 切速度档
    if (key(1, long_press)) {
        s_accel_idx[M_Ax(m)] = (s_accel_idx[M_Ax(m)] + 1) % 5;
        UI_ForceRedraw();
    }
    if (key(2, long_press)) {
        s_speed_idx[M_Ax(m)] = (s_speed_idx[M_Ax(m)] + 1) % 5;
        UI_ForceRedraw();
    }
}

static void MotorXPage_Render(void) { MotorPage_Render(SM_X); }
static void MotorYPage_Render(void) { MotorPage_Render(SM_Y); }
static void MotorXPage_OnKey(void)  { MotorPage_OnKey(SM_X); }
static void MotorYPage_OnKey(void)  { MotorPage_OnKey(SM_Y); }

/* ============================================================================
 * ==================== 灰度+TB6612 页 HuiduTBPage ====================
//
//  ROW 0: k:1 1 1 1 1    键位
//  ROW 1: 1 1 1 1 1...    二值化
//  ROW 2-5: ADC 原始值     (MUX_ADC 模式)
//  ROW 6/7: L/R 编码器
//  ROW 8: SEL:>L 60 R 40  selected + 两路 PWM 档
//  ROW 9: 页脚
//
//  按键:
//    K1: 切灰度模式
//    K2 按下: 两电机 FWD | 松开: 停
//    K3 按下: 两电机 REV | 松开: 停
//    K4 短按: 切 selected PWM 档 | 长按: 切换 selected 电机
 * ============================================================================ */
static void HuiduTBPage_Render(void) {
    char line[LCD_ROW_CHARS + 2];
    row_put_keys();

    // ROW 1 二值化
    uint8_t digital = Huidu_GetDigital();
    huidu_mode_t mode = Huidu_GetMode();
    char *p = line;
    uint8_t cnt = (mode == HUIDU_MODE_FIRST5_GPIO) ? 5 : 8;
    for (uint8_t i = 0; i < cnt; i++) {
        *p++ = (digital & (1 << i)) ? '1' : '0';
        if (i < cnt - 1) *p++ = ' ';
    }
    *p = '\0';
    row_put(1, line, WHITE, BLACK);

    // ROW 2-5 ADC 原始值
    if (mode == HUIDU_MODE_MUX_ADC) {
        uint16_t raw[8];
        Huidu_GetAnalog(raw);
        for (uint8_t row = 0; row < 4; row++) {
            uint16_t ra = raw[row * 2] > 4095 ? 4095 : raw[row * 2];
            uint16_t rb = raw[row * 2 + 1] > 4095 ? 4095 : raw[row * 2 + 1];
            snprintf(line, sizeof(line), "%u:%04u %u:%04u",
                     row * 2 + 1, (unsigned)ra, row * 2 + 2, (unsigned)rb);
            uint16_t vmax = ra > rb ? ra : rb;
            uint16_t fg = vmax > 2500 ? GREEN : (vmax < 1500 ? RED : WHITE);
            row_put(2 + row, line, fg, BLACK);
        }
    } else {
        for (uint8_t i = 2; i <= 5; i++) row_put(i, "", GRAY, BLACK);
    }

    // ROW 6/7 编码器
    snprintf(line, sizeof(line), "L:%+05ld        ", (long)Encoder_GetCountL());
    row_put(6, line, WHITE, BLACK);
    snprintf(line, sizeof(line), "R:%+05ld        ", (long)Encoder_GetCountR());
    row_put(7, line, WHITE, BLACK);

    // ROW 8 selected + PWM 档
    static const uint8_t kLvPct[3] = {20, 40, 60};
    tb_motor_t sel = TB6612_GetSelected();
    uint8_t lL = TB6612_GetLevel(TB_MOTOR_L);
    uint8_t lR = TB6612_GetLevel(TB_MOTOR_R);
    snprintf(line, sizeof(line), "SEL:%sL %u R %u",
             sel == TB_MOTOR_L ? ">" : " ",
             (unsigned)kLvPct[lL < 3 ? lL : 0],
             (unsigned)kLvPct[lR < 3 ? lR : 0]);
    row_put(8, line, WHITE, BLACK);
}

static void HuiduTBPage_OnKey(void) {
    // K1: 切灰度模式
    if (key(1, down)) { Huidu_NextMode(); UI_ForceRedraw(); }

    // K2 按下: 两电机 FWD | 松开: 停
    if (key(2, down)) {
        TB6612_Run(TB_MOTOR_L, TB_DIR_FWD);
        TB6612_Run(TB_MOTOR_R, TB_DIR_FWD);
        UI_ForceRedraw();
    }
    if (key(2, up)) {
        TB6612_Stop(TB_MOTOR_L);
        TB6612_Stop(TB_MOTOR_R);
        UI_ForceRedraw();
    }

    // K3 按下: 两电机 REV | 松开: 停
    if (key(3, down)) {
        TB6612_Run(TB_MOTOR_L, TB_DIR_REV);
        TB6612_Run(TB_MOTOR_R, TB_DIR_REV);
        UI_ForceRedraw();
    }
    if (key(3, up)) {
        TB6612_Stop(TB_MOTOR_L);
        TB6612_Stop(TB_MOTOR_R);
        UI_ForceRedraw();
    }

    // K4 短按: 切 selected PWM 档 | 长按: 切换 selected 电机
    if (key(4, down)) { TB6612_NextLevel(TB6612_GetSelected()); UI_ForceRedraw(); }
    if (key(4, long_press)) { TB6612_ToggleSelected(); UI_ForceRedraw(); }
}

/* ============================================================================
 * ==================== MPU9250 页 ====================
//
//  ROW 0: k:1 1 1 1 1
//  ROW 1: X     Y     Z     标签
//  ROW 2: a+1.20+0.34-0.98  加速度 [g]
//  ROW 3: g+1.24+4.47+0.12  陀螺 [dps] (校准中显示 cal...)
//  ROW 4: m+12.3 -4.5 +6.7  磁力 [uT] (无磁力计显示 no mag)
//  ROW 5: w+060.0+000.0 360.0 累计欧拉
//  ROW 6: Y     P     R     标签
//  ROW 7: +45.3 -12.4 +5.7  yaw/pitch/roll
//  ROW 8: T:+27.3C CAL=2   温度+校准状态
//  ROW 9: 页脚
//
//  按键: K1 短按 重新校准陀螺零漂
 * ============================================================================ */
static void MPU9250Page_Render(void) {
    char line[LCD_ROW_CHARS + 2];
    const MPU9250_Data *d = MPU9250_GetData();
    row_put_keys();

    if (!d->ok) {  // 设备未连接
        row_put(1, "X     Y    Z   ", GRAY, BLACK);
        row_put(2, "a no dev          ", GRAY, BLACK);
        row_put(3, "g no dev          ", GRAY, BLACK);
        row_put(4, "m no dev          ", GRAY, BLACK);
        row_put(5, "w no dev          ", GRAY, BLACK);
        row_put(6, "Y     P    R   ", GRAY, BLACK);
        row_put(7, "y no dev          ", GRAY, BLACK);
        row_put(8, "T:----C  --       ", GRAY, BLACK);
        return;
    }

    row_put(1, "X     Y    Z   ", WHITE, BLACK);
    snprintf(line, sizeof(line), "a%+4.1f%+4.1f%+4.1f", (double)d->ax, (double)d->ay, (double)d->az);
    row_put(2, line, WHITE, BLACK);

    if (d->calib_state == 1) {
        row_put(3, "g cal...        ", GRAY, BLACK);
    } else {
        snprintf(line, sizeof(line), "g%+4.1f%+4.1f%+4.1f", (double)d->gx, (double)d->gy, (double)d->gz);
        row_put(3, line, WHITE, BLACK);
    }

    if (d->who_mag == AK8963_WHO_AM_I_VAL) {
        snprintf(line, sizeof(line), "m%+4.1f%+4.1f%+4.1f", (double)d->mx, (double)d->my, (double)d->mz);
        row_put(4, line, WHITE, BLACK);
    } else {
        row_put(4, "m no mag        ", GRAY, BLACK);
    }

    snprintf(line, sizeof(line), "w%+5.1f%+5.1f%5.1f", (double)d->euler_x, (double)d->euler_y, (double)d->euler_z);
    row_put(5, line, YELLOW, BLACK);

    row_put(6, "Y     P    R   ", WHITE, BLACK);
    snprintf(line, sizeof(line), "%+5.1f%+5.1f%+5.1f", (double)d->yaw, (double)d->pitch, (double)d->roll);
    row_put(7, line, d->who_mag == AK8963_WHO_AM_I_VAL ? WHITE : GRAY, BLACK);

    snprintf(line, sizeof(line), "T:%+5.1fC CAL=%1u", (double)d->temp_c, (unsigned)d->calib_state);
    row_put(8, line, CYAN, BLACK);
}

static void MPU9250Page_OnKey(void) {
    if (key(1, down)) { MPU9250_ResetGyroCalib(); UI_ForceRedraw(); }
}

/* ============================================================================
 * ==================== 页面注册表 ====================
 * ============================================================================ */
const page_t pages[] = {
    { "MENU",    MenuPage_Render,    MenuPage_OnKey,    NULL },   // [0] 菜单
    { "main",    MainPage_Render,    MainPage_OnKey,    NULL },   // [1] 主页
    { "TASK1",   Task1Page_Render,   TaskPage_OnKey,    NULL },   // [2]
    { "TASK2",   Task2Page_Render,   TaskPage_OnKey,    NULL },   // [3]
    { "TASK3",   Task3Page_Render,   TaskPage_OnKey,    NULL },   // [4]
    { "TASK4",   Task4Page_Render,   TaskPage_OnKey,    NULL },   // [5]
    { "TASK5",   Task5Page_Render,   TaskPage_OnKey,    NULL },   // [6]
    { "MOTOR-X", MotorXPage_Render,  MotorXPage_OnKey,  NULL },   // [7] X 轴
    { "MOTOR-Y", MotorYPage_Render,  MotorYPage_OnKey,  NULL },   // [8] Y 轴
    { "HUITB",   HuiduTBPage_Render, HuiduTBPage_OnKey, NULL },   // [9] 灰度+TB6612
    { "MPU9250", MPU9250Page_Render, MPU9250Page_OnKey, NULL },   // [10] IMU
};

/* ============================================================================
 * ==================== 公开 API ====================
 * ============================================================================ */
void UI_Init(void) {
    g_menu_sel = MENU_BOOT_SEL;
    mark_all_dirty();
}

void UI_ForceRedraw(void) {
    mark_all_dirty();
}

void UI_Render(void) {
    pages[g_page].render();
    if (pages[g_page].footer_hook) {
        pages[g_page].footer_hook();
    } else {
        char line[20];
        snprintf(line, sizeof(line), "P%d/%d %s",
                 g_page + 1, PAGE_COUNT, pages[g_page].name);
        row_put(9, line, WHITE, DARKBLUE);
    }
    row_flush();
}

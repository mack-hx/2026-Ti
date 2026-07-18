/* ============================================================================
 * @file    ui.c
 * @brief   LCD 多页面 UI (128x160 竖屏, 8x16 字体, 行高 16 px, 共 10 行)
 *
 *   两层架构:
 *     - g_page == 0          → 菜单页 (MenuPage)
 *     - g_page == 1..PAGE_COUNT-1 → 详情页 (Task1..5 / Motor-X/Y / Huitb / MPU9250)
 *
 *   渲染框架 (顶部):
 *     - row_put / row_flush / UI_ForceRedraw — 16-bit FNV-1a 行级脏位
 *     - pages[] — 注册表, 由 render/on_key/footer_hook 三个函数指针构成
 *
 *   页面业务逻辑 (中下部):
 *     - 每页分成 '=== Page: <name> ===' 块, 一块占几十行
 *     - 改页行为: 找对应块, 改 Key/Display 即可
 *     - 改页参数: 顶部宏 (MENU_NAMES / MOTOR_* / LCD_ROW_CHARS 等)
 *
 *   主循环调用 (见 main.c):
 *     UI_Init();       // 上电
 *     UI_Render();     // 每帧
 *     UI_ForceRedraw();// 切页 / 按键触发
 *
 *   颜色约定 (与 LCD.h 一致):
 *     WHITE=普通  YELLOW=选中  GREEN=成功  RED=错误  GRAY=占位
 *     CYAN=调试    DARKBLUE=标题/页脚底色
 *
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
 * 顶部参数表 — 改这里调整页面行为 (不接触底层 API)
 * ============================================================================ */

/* 移动精度档 (短按 K4 循环, 0.1° 单位) */
static const uint16_t kMotorStepTenths[] = { 10U, 30U, 50U, 100U, 300U, 600U, 900U, 1U };

/* 移动速度档 (RPM, 长按 K2 循环) */
static const uint16_t kMotorSpeedRpm[]   = { 7U, 10U, 15U, 25U, 40U };

/* 移动加速度档 (0~200, 长按 K1 循环) */
static const uint8_t kMotorAccel[]        = { 10U, 15U, 20U, 30U, 50U };

/* 上电默认进 main 页 (g_page=1) — 主页占 [1], 启动即进
 *
 *   菜单 g_menu_sel 默认跳到 TASK1 (下标 2), 让菜单页 K5 第一次进 TASK1
 *   而不是回到 main; 想回 main 在菜单里按 K2 (sel 上移 / 循环到末尾)
 *   然后 K5 进入 */
#ifndef MAIN_DEFAULT_PAGE
#define MAIN_DEFAULT_PAGE  1U
#endif

/* 菜单默认选中 TASK1 (对应 pages[2]) — main 占 [1] 不在菜单里 */
#define MAIN_BOOT_MENU_SEL  2U
#define MOTOR_STEP_DEFAULT_IDX  3U   /* 10°/次 */
#define MOTOR_SPEED_DEFAULT_IDX 2U   /* 15 RPM */
#define MOTOR_ACCEL_DEFAULT_IDX 2U   /* 20 */

/* 步进电机零点参数 (硬件固定不变, K4 长按一次性写入) */
#define MOTOR_PULSES_PER_REV         51200L
#define MOTOR_X_LEFT_DEG             0
#define MOTOR_X_RIGHT_DEG            1800
#define MOTOR_Y_LEFT_DEG             0
#define MOTOR_Y_RIGHT_DEG            220
#define MOTOR_ZEROSET_TIMEOUT_MS     10000U
#define MOTOR_ZEROSET_AUTO_HOME      true
#define MOTOR_ZEROSET_LIMIT_ON       true

/* 度 → 脉冲 (四舍五入) */
#define MOTOR_DEG_TO_PULSES(deg)  \
    ((int32_t)(((int64_t)(deg) * (int64_t)MOTOR_PULSES_PER_REV + 180LL) / 360LL))

/* ============================================================================
 * 全局状态 (main.c 通过 g_page / g_menu_sel 切页)
 * ============================================================================ */
uint8_t       g_page     = MAIN_DEFAULT_PAGE;
uint8_t       g_menu_sel = MAIN_BOOT_MENU_SEL;
const uint8_t PAGE_COUNT = 11;

/* 每轴独立的档位状态 (X/Y 各一组, 切换页不影响) */
static uint8_t s_motor_step_index[2]  = { MOTOR_STEP_DEFAULT_IDX, MOTOR_STEP_DEFAULT_IDX };
static uint8_t s_motor_speed_index[2] = { MOTOR_SPEED_DEFAULT_IDX, MOTOR_SPEED_DEFAULT_IDX };
static uint8_t s_motor_accel_index[2] = { MOTOR_ACCEL_DEFAULT_IDX, MOTOR_ACCEL_DEFAULT_IDX };

/* 主页 K1 toggle 启停 TB6612 两电机的状态 (用户 2026-07-18 06:02 决定)
 *   false = 静止 (上电默认),  K1 down → 两电机 Run(FWD, 当前 PWM 档) + 置 true
 *   true  = 在转 (小车前进),  K1 down → 两电机 Stop + 置 false
 * 注: 仅在 MainPage 内部维护, 切到其它页 K5 不联动, 防止状态泄漏.
 *     跟 HuiduTBPage 的 K2/K3 (独立 Run FWD/REV) 不冲突, 它们的 page-local 状态机
 *     在切换时会自然重置 (Run/Stop 总是按当前档 compare 应用). */
static bool s_main_k1_tb_running = false;

/* ============================================================================
 * 行级脏位渲染框架
 *
 *   每行缓存 last_text + FNV-1a 16-bit 哈希, 内容不变 → 跳过 LCD 写入。
 *   切页时调 mark_all_rows_dirty() 强制整页重画。
 * ============================================================================ */
#define LCD_ROWS  10U

typedef struct {
    char     text[LCD_ROW_CHARS + 2];
    uint16_t hash;
    uint16_t fg;
    uint16_t bg;
    uint8_t  dirty;
} row_buf_t;

static row_buf_t s_rows[LCD_ROWS];
static uint8_t   s_row_count = 0;

static uint16_t fnv1a16(const char *s) {
    uint16_t h = 21661u;
    for (; *s; s++) {
        h = (uint16_t)((h ^ (uint8_t)*s) * 16777u);
    }
    return h;
}

static void row_put(uint8_t idx, const char *txt, uint16_t fg, uint16_t bg) {
    if (idx >= LCD_ROWS) return;

    /* 严格截断到 16 字符 (调用方也应用 snprintf 宽度限定符) */
    char buf[LCD_ROW_CHARS + 2];
    uint8_t i = 0;
    for (; i < LCD_ROW_CHARS && txt[i] != '\0'; i++) {
        buf[i] = txt[i];
    }
    buf[i] = '\0';

    uint16_t h = fnv1a16(buf);
    row_buf_t *r = &s_rows[idx];
    if (!r->dirty && r->hash == h && strcmp(r->text, buf) == 0) {
        if (idx + 1U > s_row_count) s_row_count = (uint8_t)(idx + 1U);
        return;
    }
    memcpy(r->text, buf, (size_t)i + 1u);
    r->hash  = h;
    r->fg    = fg;
    r->bg    = bg;
    r->dirty = 1;
    if (idx + 1U > s_row_count) s_row_count = (uint8_t)(idx + 1U);
}

static void row_flush(void) {
    for (uint8_t i = 0; i < s_row_count; i++) {
        row_buf_t *r = &s_rows[i];
        if (!r->dirty) continue;
        uint16_t y = (uint16_t)(i * 16u);
        LCD_Fill(0, y, LCD_W, (uint16_t)(y + 16u), r->bg);
        LCD_ShowString(0, y, r->text, r->fg, r->bg, LCD_8X16, 0);
        r->dirty = 0;
    }
}

static void mark_all_rows_dirty(void) {
    for (uint8_t i = 0; i < LCD_ROWS; i++) {
        s_rows[i].dirty = 1;
        s_rows[i].text[0] = '\0';
        s_rows[i].hash = 0;
    }
    s_row_count = 0;
}

/* ============================================================================
 * 公共小工具: ROW 0 键位条 "k:1 1 1 1 1"
 * ============================================================================ */
static void row_put_keys(void) {
    char line[LCD_ROW_CHARS + 2];
    snprintf(line, sizeof(line), "k:%u %u %u %u %u",
             key_pressed(1) ? 0U : 1U, key_pressed(2) ? 0U : 1U,
             key_pressed(3) ? 0U : 1U, key_pressed(4) ? 0U : 1U,
             key_pressed(5) ? 0U : 1U);
    row_put(0, line, WHITE, BLACK);
}

/* 灰度二值化行格式化 (按模式画 5 位 / 8 位, 主页和 HuiduTBPage 共用)
 *   实现见下方 HuiduTBPage 块 (此为前向声明, 主页先用到) */
static void FormatHuiduDigital(char *line, uint8_t digital, huidu_mode_t mode);

/* ============================================================================
 * Page: Menu (g_page == 0)
 *
 *   ROW 0    "   == MENU ==   "  标题
 *   ROW 1..8 8 个菜单项 (选中项前缀 '>' + YELLOW)
 *   ROW 9    缺省页脚 "P1/9 MENU"
 *
 *   按键路由在 main.c (g_menu_sel 切换 / g_page = g_menu_sel), 不在 on_key。
 * ============================================================================ */
static const char * const kMenuNames[MENU_ITEM_COUNT] = {
    "MAIN", "TASK1", "TASK2", "TASK3", "TASK4", "TASK5",
    "MOTOR-X", "MOTOR-Y", "HUITB", "MPU9250",
};

static void MenuPage_Render(void) {
    row_put(0, "   == MENU ==   ", WHITE, DARKBLUE);

    char line[LCD_ROW_CHARS + 2];
    /* 9 项菜单只有 8 行空间, 选中 ≥6 项时窗口下移一行 (滚动到末尾 8 项) */
    uint8_t first = (g_menu_sel > 8U) ? (uint8_t)(g_menu_sel - 7U) : 1U;
    for (uint8_t row = 0U; row < 8U; row++) {
        uint8_t sel = (uint8_t)(first + row);
        snprintf(line, sizeof(line), "%c%-15.15s",
                 (g_menu_sel == sel) ? '>' : ' ',
                 kMenuNames[sel - 1U]);
        row_put((uint8_t)(1U + row), line,
                (g_menu_sel == sel) ? YELLOW : WHITE, BLACK);
    }
}

static void MenuPage_OnKey(void) {
    /* K1/K2/K5 路由全部在 main.c (跟 g_page 切换耦合), 不重复处理 */
}

/* ============================================================================
 * Page: Main (g_page == 1) — 主页: 所有外设关键状态总览
 *
 *   上电默认进入此页 (g_page=MAIN_DEFAULT_PAGE=1), K5 切到菜单.
 *
 *   ROW 0   k:1 1 1 1 1         5 路按键当前电平 (1=松开, 0=按下)
 *   ROW 1   0 0 0 0 0 0 0 0     灰度 8 路二值化 (按当前模式画 5/8 位)
 *   ROW 2   X:+12345Y:-6789     PD42S1 X/Y 轴位置 (X/Y 错开 300ms 各读一次)
 *   ROW 3   SPX:60RPM AC:100    X 轴速度档 + 加速度档
 *   ROW 4   SPY:60RPM AC:100    Y 轴速度档 + 加速度档
 *   ROW 5   L:+12345R:-6789     编码器 L/R 累计值 (无空格)
 *   ROW 6   PWM:>Axxxx Bxxxx    TB6612 两路实际 compare 值; running 时 '>' 前缀 + GREEN
 *   ROW 7   +1.00+2.00-0.50     陀螺仪三轴 (dps, 无空格)
 *   ROW 8   K230:+123,+1280     K230 通信: 有坐标 (绿) / no rec (红) / no link (灰)
 *   ROW 9   P1/11 main          页脚
 *
 *   按键语义 (用户 2026-07-18 06:02 决定):
 *     K1 down → toggle TB6612 两电机 FWD (用当前 PWM 档, 小车前进)
 *     K2 down/up → K230 激光开/关 (06:00 已加)
 *     K3/K4 → no-op
 *     K5 由 main.c 切页 (不联动停 TB6612, 由用户 toggle K1 或在 HuiduTB 页关)
 * ============================================================================ */
static void MainPage_Render(void) {
    char line[LCD_ROW_CHARS + 2];

    /* ROW 0 键位条 (跟其它页一致: 1=松开, 0=按下) */
    snprintf(line, sizeof(line), "k:%u %u %u %u %u",
             key_pressed(1) ? 0U : 1U, key_pressed(2) ? 0U : 1U,
             key_pressed(3) ? 0U : 1U, key_pressed(4) ? 0U : 1U,
             key_pressed(5) ? 0U : 1U);
    row_put(0, line, WHITE, BLACK);

    /* ROW 1 灰度二值化 (按当前模式画 5 位 / 8 位)
     *   默认 ALL8_GPIO: "0 0 0 0 0 0 0 0" = 15 字符
     *   FIRST5_GPIO : "0 0 0 0 0" = 9 字符
     *   跟 HuiduTBPage 用同一个 FormatHuiduDigital, 模式切时主页 ROW 1 跟随同步 */
    {
        uint8_t digital = Huidu_GetDigital();
        huidu_mode_t mode = Huidu_GetMode();
        FormatHuiduDigital(line, digital, mode);
        row_put(1, line, WHITE, BLACK);
    }

    /* ROW 2 步进电机 X+Y 位置 (合并一行, 不带 "SM-X:" 前缀)
     *   例: "X:+12345Y:-6789" = 16 字符 (各占 7 字符: "X:" + 6 位含符号)
     *   2026-07-18: 去掉中间空格修 "超 1 位" 问题 (用户反馈). */
    {
        int32_t pos_x = SM_GetPosition(SM_X);
        int32_t pos_y = SM_GetPosition(SM_Y);
        snprintf(line, sizeof(line), "X:%+6ldY:%+6ld",
                 (long)pos_x, (long)pos_y);
        row_put(2, line, WHITE, BLACK);
    }

    /* ROW 3 / ROW 4 X / Y 轴速度档 + 加速度档 (各自独立, 跟 motor 页 K1/K2 长按档位同步)
     *   例: "SPX:60RPM AC:100" = 15 字符 (rpm 最大 60, accel 最大 100)
     *   SP 前缀后加 X/Y 一眼分清两轴; accel 用 %3u 留扩展空间 */
    for (uint8_t i = 0U; i < 2U; i++) {
        uint8_t axis = i;
        uint8_t s_idx = s_motor_speed_index[axis];
        uint8_t a_idx = s_motor_accel_index[axis];
    uint16_t rpm = kMotorSpeedRpm[s_idx < 5U ? s_idx : 2U];
    uint8_t accel = kMotorAccel[a_idx < 5U ? a_idx : 2U];
        snprintf(line, sizeof(line), "SP%c:%2uRPM AC:%3u",
                 (i == 0U) ? 'X' : 'Y',
                 (unsigned)rpm, (unsigned)accel);
        row_put((uint8_t)(3U + i), line, YELLOW, BLACK);
    }

    /* ROW 5 编码器 L/R 累计值
     *   例: "L:+12345R:-6789" = 16 字符 (各占 7 字符: "L:" + 6 位含符号)
     *   2026-07-18: 去掉中间空格修 "超 1 位" 问题 (用户反馈). */
    {
        int32_t l = Encoder_GetCountL();
        int32_t r = Encoder_GetCountR();
        snprintf(line, sizeof(line), "L:%+6ldR:%+6ld", (long)l, (long)r);
        row_put(5, line, WHITE, BLACK);
    }

    /* ROW 6 TB6612 两路实际 compare 值 + 主页 K1 toggle 启停状态
     *   静止 (默认): "PWM:A 400B 800"  14 字符, WHITE
     *   在转 (K1 toggle on): "PWM:>A 400 B 800"  16 字符, GREEN + '>' 提示
     *   compare 范围 0~2000 (% 对应 0~100, 但 PWM 档 20/40/60 对应 400/800/1200) */
    {
        uint16_t ca = TB6612_GetCompare(TB_MOTOR_L);
        uint16_t cb = TB6612_GetCompare(TB_MOTOR_R);
        const char prefix = s_main_k1_tb_running ? '>' : ' ';
        snprintf(line, sizeof(line), "PWM:%cA%-4u B%-4u",
                 (int)prefix, (unsigned)ca, (unsigned)cb);
        row_put(6, line, s_main_k1_tb_running ? GREEN : WHITE, BLACK);
    }

    /* ROW 7 陀螺仪三轴 (dps)
     *   例: "+1.00+2.00-0.50" (三轴值无空格拼接) */
    {
        const MPU9250_Data *d = MPU9250_GetData();
        if (d->ok) {
            snprintf(line, sizeof(line), "%+5.2f%+5.2f%+5.2f",
                     (double)d->gx, (double)d->gy, (double)d->gz);
            row_put(7, line, WHITE, BLACK);
        } else {
            snprintf(line, sizeof(line), "G: no dev       ");
            row_put(7, line, GRAY, BLACK);
        }
    }

    /* ROW 8 K230 通信信息
     *   - 有坐标时 (center_point 或实时坐标): "{x},{y}" (含符号, 一律 5 位数宽 → 字符串 ≤12 字符)
     *   - 识别失败 ("识别错误" 字串): "no rec"
     *   - 完全没收到 (上电 / K230 未连): "no link" (灰)
     *   例: "K230:+ 123,+45 " = 16 字符 (各占 6 字符: 含符号 5 位 + ',' / 空格)
     *       "K230:+123,+1280 " = 16 字符
     *       "K230:no rec      " = 17 字符 (截断到 16)
     * 优先级: has_coord (绿) > !recognition_ok 红 > no link 灰.
     * 注: 上电 g_k230_cmd={0} → "no link" (灰, ✓). */
    {
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
}

static void MainPage_OnKey(void) {
    /* K1 down → toggle 启停 TB6612 两电机 (FWD + 当前 PWM 档, 小车前进)
     *   K1 长按 → no-op (motor 页里 K1 长按是切加速度档, 主页里没这概念)
     *   K2 down → 给 K230 发"激光打开" (K230 端识别字串后 PWM3 满占空比 + 进 flag=4)
     *   K2 up   → 发"激光关闭"  (PWM3 置零, 退出 flag=4)
     * 帧尾带 \r\n 让 K230 端的 UART 收包状态机识别为完整一帧.
     * K3/K4 仍 no-op. K5 由 main.c 在更早的 handle_navigation_keys() 里接走. */
    if (key(1, down)) {
        if (s_main_k1_tb_running) {
            TB6612_Stop(TB_MOTOR_L);
            TB6612_Stop(TB_MOTOR_R);
            s_main_k1_tb_running = false;
        } else {
            TB6612_Run(TB_MOTOR_L, TB_DIR_FWD);
            TB6612_Run(TB_MOTOR_R, TB_DIR_FWD);
            s_main_k1_tb_running = true;
        }
        UI_ForceRedraw();
    } else if (key(2, down)) {
        UART_SendString(UART_CH1, "激光打开\r\n");
        UI_ForceRedraw();
    } else if (key(2, up)) {
        UART_SendString(UART_CH1, "激光关闭\r\n");
        UI_ForceRedraw();
    }
}

static void TaskPage_Render(uint8_t n) {
    row_put_keys();
    char line[LCD_ROW_CHARS + 2];
    snprintf(line, sizeof(line), "     TASK%u     ", (unsigned)n);
    row_put(1, line, YELLOW, BLACK);
    row_put(2, "  to be added   ", GRAY, BLACK);
    for (uint8_t i = 3; i <= 8; i++) row_put(i, "", GRAY, BLACK);
}

static void TaskPage_OnKey(void) {}

static void Task1Page_Render(void) { TaskPage_Render(1U); }
static void Task2Page_Render(void) { TaskPage_Render(2U); }
static void Task3Page_Render(void) { TaskPage_Render(3U); }
static void Task4Page_Render(void) { TaskPage_Render(4U); }
static void Task5Page_Render(void) { TaskPage_Render(5U); }

/* ============================================================================
 * Page: Motor-X / Motor-Y (g_page == 7, 8) — PD42S1 闭环步进电机调试
 *
 *   ROW 0   k:1 1 1 1 1              键位
 *   ROW 1   AX:X STEP:1.0            轴 + 当前步长档 (YELLOW)
 *   ROW 2   POS:+12345               当前位置 [脉冲]
 *   ROW 3   DEG:+123.45              当前位置 [度, 2 位小数] (GREEN)
 *   ROW 4   T:CC 55 01 FA 01 ...     TX 原始 hex (前 7 字节, CYAN)
 *   ROW 5   T:<剩 8 字节>             TX 续
 *   ROW 6   R:CC 55 01 2A 01 ...     RX 原始 hex (含 ERR 字节, GREEN/RED)
 *   ROW 7   R:<剩 8 字节>             RX 续
 *   ROW 8   S: 30RPM A: 50           当前速度档 + 加速度档 (YELLOW)
 *   ROW 9   页脚
 *
 *   按键:
 *     K1 down  → SM_Move(R)  正转 (当前档)         | K1 long_press  切加速度档
 *     K2 down  → SM_Move(L)  反转                  | K2 long_press  切速度档
 *     K3 down  → SM_ReadPosition 刷新位置           | K3 long_press  触发驱动器回零 (SM_zero HM)
 *     K4 up    → 切精度档                          | K4 long_press  一次性写入 zeroset 6 帧
 *     K5       → main.c 切页
 * ============================================================================ */

static uint8_t MotorAxisIndex(sm_motor_t motor) {
    return (motor == SM_Y) ? 1U : 0U;
}

static int32_t MotorStepPulses(sm_motor_t motor) {
    uint8_t axis = MotorAxisIndex(motor);
    uint16_t tenths = kMotorStepTenths[s_motor_step_index[axis]];
    return (int32_t)(((uint32_t)tenths * (uint32_t)MOTOR_PULSES_PER_REV + 1800U) / 3600U);
}

static uint16_t MotorSpeed(sm_motor_t motor) {
    uint8_t axis = MotorAxisIndex(motor);
    uint8_t idx  = s_motor_speed_index[axis];
    return kMotorSpeedRpm[idx < 5U ? idx : 0U];
}

static uint8_t MotorAccel(sm_motor_t motor) {
    uint8_t axis = MotorAxisIndex(motor);
    uint8_t idx  = s_motor_accel_index[axis];
    return kMotorAccel[idx < 5U ? idx : 0U];
}

/* 把 pd42_frame_t 反推为原始字节 (head+addr+func+data...+checksum+tail) */
static uint8_t MotorFrameToRaw(pd42_frame_t *rx, uint8_t *out) {
    uint8_t n = 0U;
    out[n++] = PD42S1_FRAME_HEAD;
    out[n++] = rx->slave_addr;
    out[n++] = rx->function_code;
    for (uint8_t i = 0U; i < rx->data_len && n < 8U; i++) {
        out[n++] = rx->data[i];
    }
    out[n++] = (uint8_t)rx->checksum;
    out[n++] = PD42S1_FRAME_TAIL;
    return n;
}

/* TX/RX 原始 hex 兏底 (拆两行, 每行最多 7 字节 = 14 字符) */
static void MotorPutHexRows(uint8_t first_row, char prefix,
                           const volatile uint8_t *data, uint8_t len,
                           uint16_t color) {
    char line[LCD_ROW_CHARS + 2];
    if (data == NULL || len == 0U) {
        snprintf(line, sizeof(line), "%c:------------", prefix);
        row_put(first_row, line, GRAY, BLACK);
        row_put((uint8_t)(first_row + 1U), "", GRAY, BLACK);
        return;
    }
    uint8_t first = (len > 7U) ? 7U : len;
    snprintf(line, sizeof(line), "%c:", prefix);
    char *p = line + 2;
    for (uint8_t i = 0U; i < first; i++) {
        p += snprintf(p, 3, "%02X", (unsigned)data[i]);
    }
    *p = '\0';
    row_put(first_row, line, color, BLACK);

    p = line;
    uint8_t remain = (len > 7U) ? (uint8_t)(len - 7U) : 0U;
    if (remain > 8U) remain = 8U;
    for (uint8_t i = 0U; i < remain; i++) {
        p += snprintf(p, 3, "%02X", (unsigned)data[7U + i]);
    }
    *p = '\0';
    row_put((uint8_t)(first_row + 1U), line, color, BLACK);
}

static void MotorZeroset(sm_motor_t motor) {
    if (motor == SM_X) {
        SM_zeroset(SM_X,
                   MOTOR_DEG_TO_PULSES(MOTOR_X_LEFT_DEG),
                   MOTOR_DEG_TO_PULSES(MOTOR_X_RIGHT_DEG),
                   MOTOR_ZEROSET_TIMEOUT_MS,
                   MOTOR_ZEROSET_AUTO_HOME, MOTOR_ZEROSET_LIMIT_ON);
    } else {
        SM_zeroset(SM_Y,
                   MOTOR_DEG_TO_PULSES(MOTOR_Y_LEFT_DEG),
                   MOTOR_DEG_TO_PULSES(MOTOR_Y_RIGHT_DEG),
                   MOTOR_ZEROSET_TIMEOUT_MS,
                   MOTOR_ZEROSET_AUTO_HOME, MOTOR_ZEROSET_LIMIT_ON);
    }
}

static void MotorPage_Render(sm_motor_t motor) {
    char line[LCD_ROW_CHARS + 2];
    uint8_t axis = MotorAxisIndex(motor);

    row_put_keys();

    /* ROW 1 轴 + 当前步长档 */
    uint16_t tenths = kMotorStepTenths[s_motor_step_index[axis]];
    snprintf(line, sizeof(line), "AX:%c STEP:%4.1f",
             (motor == SM_X) ? 'X' : 'Y', (double)tenths / 10.0);
    row_put(1, line, YELLOW, BLACK);

    /* ROW 2/3 位置 */
    int32_t pos = SM_GetPosition(motor);
    double degree = ((double)pos * 360.0) / (double)MOTOR_PULSES_PER_REV;
    snprintf(line, sizeof(line), "POS:%+11ld", (long)pos);
    row_put(2, line, WHITE, BLACK);
    snprintf(line, sizeof(line), "DEG:%+12.2f", degree);
    row_put(3, line, GREEN, BLACK);

    /* ROW 4-5 TX 原始 hex */
    uint8_t tx_len = 0U;
    bool tx_fired = false;
    const volatile uint8_t *tx = PD42S1_GetTxBuffer((uint8_t)motor, &tx_len, &tx_fired);
    MotorPutHexRows(4U, 'T', tx, tx_fired ? tx_len : 0U, CYAN);

    /* ROW 6-7 RX 原始 hex (含 ERR 字节染色) */
    pd42_frame_t *rx = PD42S1_GetFrameFor((uint8_t)motor);
    uint8_t rx_raw[10];
    uint8_t rx_len = 0U;
    if (rx->function_code != 0U) {
        rx_len = MotorFrameToRaw(rx, rx_raw);
    }
    uint16_t rx_color = (rx_len > 0U && rx->error_code == PD42_ACK_OK) ? GREEN : RED;
    MotorPutHexRows(6U, 'R', rx_raw, rx_len, rx_color);

    /* ROW 8 当前速度档 + 加速度档 */
    uint8_t s_idx = s_motor_speed_index[axis];
    uint8_t a_idx = s_motor_accel_index[axis];
    snprintf(line, sizeof(line), "S:%3uRPM A:%3u",
             (unsigned)kMotorSpeedRpm[s_idx < 5U ? s_idx : 0U],
             (unsigned)kMotorAccel[a_idx < 5U ? a_idx : 0U]);
    row_put(8, line, YELLOW, BLACK);
}

static void MotorPage_OnKey(sm_motor_t motor) {
    int32_t pulses = MotorStepPulses(motor);
    uint16_t rpm   = MotorSpeed(motor);
    uint8_t  accel = MotorAccel(motor);

    if (key(1, down)) {
        SM_Move(motor, R, accel, rpm, pulses);
        UI_ForceRedraw();
    }
    if (key(1, long_press)) {
        uint8_t axis = MotorAxisIndex(motor);
        s_motor_accel_index[axis] = (uint8_t)((s_motor_accel_index[axis] + 1U) % 5U);
        UI_ForceRedraw();
    }
    if (key(2, down)) {
        SM_Move(motor, L, accel, rpm, pulses);
        UI_ForceRedraw();
    }
    if (key(2, long_press)) {
        uint8_t axis = MotorAxisIndex(motor);
        s_motor_speed_index[axis] = (uint8_t)((s_motor_speed_index[axis] + 1U) % 5U);
        UI_ForceRedraw();
    }
    if (key(3, down)) {
        SM_ReadPosition(motor);
        UI_ForceRedraw();
    }
    if (key(3, long_press)) {
        SM_zero(motor, HM);
        UI_ForceRedraw();
    }
    /* K4 短按只能在 up 触发, 否则 long_press 触发时会先发 down 把档切了 */
    if (key(4, up)) {
        uint8_t axis = MotorAxisIndex(motor);
        s_motor_step_index[axis] = (uint8_t)((s_motor_step_index[axis] + 1U) % 8U);
        UI_ForceRedraw();
    }
    if (key(4, long_press)) {
        MotorZeroset(motor);
        UI_ForceRedraw();
    }
}

static void MotorXPage_Render(void) { MotorPage_Render(SM_X); }
static void MotorYPage_Render(void) { MotorPage_Render(SM_Y); }
static void MotorXPage_OnKey(void)  { MotorPage_OnKey(SM_X); }
static void MotorYPage_OnKey(void)  { MotorPage_OnKey(SM_Y); }

/* ============================================================================
 * Page: HuiduTB (g_page == 9) — 灰度 + TB6612 编码电机
 *
 *   ROW 0   k:1 1 1 1 1              键位
 *   ROW 1   1 1 1 1 1 1 1 1          二值化 (按当前模式画 5 位/8 位)
 *   ROW 2   1:XXXX 2:XXXX           ADC 原始值 CH1/2 (MUX_ADC 模式)
 *   ROW 3   3:XXXX 4:XXXX           CH3/4
 *   ROW 4   5:XXXX 6:XXXX           CH5/6
 *   ROW 5   7:XXXX 8:XXXX           CH7/8 (GPIO 模式 → 空行)
 *   ROW 6   L:+12345                左编码器
 *   ROW 7   R:+12345                右编码器
 *   ROW 8   SEL:>L 60 R 40          当前 selected + 两路 PWM 档 (%)
 *   ROW 9   页脚
 *
 *   按键:
 *     K1 down  → Huidu_NextMode (MUX_ADC → FIRST5_GPIO → ALL8_GPIO → ...)
 *     K2 down/up → 两电机 Run FWD / Stop
 *     K3 down/up → 两电机 Run REV / Stop
 *     K4 down  → NextLevel(selected) 切 PWM 档
 *     K4 long_press → ToggleSelected() 切 L/R
 * ============================================================================ */
static void FormatHuiduDigital(char *line, uint8_t digital, huidu_mode_t mode) {
    char *p = line;
    uint8_t count = (mode == HUIDU_MODE_FIRST5_GPIO) ? 5U : 8U;
    for (uint8_t i = 0; i < count; i++) {
        uint8_t bit = (uint8_t)(1U << i);
        *p++ = (digital & bit) ? '1' : '0';
        if (i < (uint8_t)(count - 1U)) *p++ = ' ';
    }
    *p = '\0';
}

static void HuiduTBPage_Render(void) {
    char line[LCD_ROW_CHARS + 2];

    row_put_keys();

    /* ROW 1 二值化 */
    uint8_t digital = Huidu_GetDigital();
    huidu_mode_t mode = Huidu_GetMode();
    FormatHuiduDigital(line, digital, mode);
    row_put(1, line, WHITE, BLACK);

    /* ROW 2-5 ADC 原始值 */
    if (mode == HUIDU_MODE_MUX_ADC) {
        uint16_t raw[8];
        Huidu_GetAnalog(raw);
        for (uint8_t row = 0; row < 4; row++) {
            uint8_t a = (uint8_t)(row * 2U);
            uint8_t b = (uint8_t)(a + 1U);
            uint16_t ra = (raw[a] > 4095U) ? 4095U : raw[a];
            uint16_t rb = (raw[b] > 4095U) ? 4095U : raw[b];
            snprintf(line, sizeof(line), "%u:%04u %u:%04u",
                     (unsigned)(a + 1U), (unsigned)ra,
                     (unsigned)(b + 1U), (unsigned)rb);
            uint16_t vmax = (ra > rb) ? ra : rb;
            uint16_t fg = (vmax > 2500U) ? GREEN : (vmax < 1500U) ? RED : WHITE;
            row_put((uint8_t)(2 + row), line, fg, BLACK);
        }
    } else {
        for (uint8_t row = 2; row <= 5; row++) row_put(row, "", GRAY, BLACK);
    }

    /* ROW 6/7 编码器 */
    snprintf(line, sizeof(line), "L:%+05ld        ", (long)Encoder_GetCountL());
    row_put(6, line, WHITE, BLACK);
    snprintf(line, sizeof(line), "R:%+05ld        ", (long)Encoder_GetCountR());
    row_put(7, line, WHITE, BLACK);

    /* ROW 8 SEL + 两路 PWM 档 (%) */
    static const uint8_t kLvPct[3] = { 20, 40, 60 };
    tb_motor_t sel = TB6612_GetSelected();
    uint8_t lL = TB6612_GetLevel(TB_MOTOR_L);
    uint8_t lR = TB6612_GetLevel(TB_MOTOR_R);
    snprintf(line, sizeof(line), "SEL:%sL %u R %u",
             (sel == TB_MOTOR_L) ? ">" : " ",
             (unsigned)kLvPct[lL < 3U ? lL : 0U],
             (unsigned)kLvPct[lR < 3U ? lR : 0U]);
    row_put(8, line, WHITE, BLACK);
}

static void HuiduTBPage_OnKey(void) {
    if (key(1, down)) { Huidu_NextMode(); UI_ForceRedraw(); }
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
    if (key(4, down)) {
        TB6612_NextLevel(TB6612_GetSelected());
        UI_ForceRedraw();
    }
    if (key(4, long_press)) {
        TB6612_ToggleSelected();
        UI_ForceRedraw();
    }
}

/* ============================================================================
 * Page: MPU9250 (g_page == 10) — 9 轴 IMU (加速度 + 陀螺 + 磁力计)
 *
 *   ROW 0   k:1 1 1 1 1
 *   ROW 1   X       Y       Z        XYZ 标签
 *   ROW 2   a+1.20+0.34-0.98        acc [g]
 *   ROW 3   g+1.24+4.47+0.12        gyro [dps] (校准中显示 "cal...")
 *   ROW 4   m+12.3 -4.5 +6.7        mag [uT] (没磁力计显示 "no mag")
 *   ROW 5   w+060.0+000.0 360.0     累计欧拉 (纯陀螺积分, 不抗漂)
 *   ROW 6   Y       P       R        姿态标签
 *   ROW 7   +45.3 -12.4 +5.7        yaw / pitch / roll [deg]
 *   ROW 8   T:+27.3C CAL=2          温度 + 校准状态
 *   ROW 9   页脚
 *
 *   按键:
 *     K1 down  → MPU9250_ResetGyroCalib() (重置陀螺零漂, 板子必须静止 1s)
 *     K2..K4   → no-op (hard-iron / yaw reset 预留位)
 * ============================================================================ */
static void MPU9250Page_Render(void) {
    char line[LCD_ROW_CHARS + 2];
    const MPU9250_Data *d = MPU9250_GetData();

    row_put_keys();

    /* 设备没接 / Init 失败: 所有数据行 → "no dev" */
    if (!d->ok) {
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

    /* ROW 1 XYZ 标签 / ROW 2 加速度 */
    row_put(1, "X     Y    Z   ", WHITE, BLACK);
    snprintf(line, sizeof(line), "a%+4.1f%+4.1f%+4.1f",
             (double)d->ax, (double)d->ay, (double)d->az);
    row_put(2, line, WHITE, BLACK);

    /* ROW 3 陀螺 (校准中显示 "cal...") */
    if (d->calib_state == 1) {
        snprintf(line, sizeof(line), "g cal...        ");
    } else {
        snprintf(line, sizeof(line), "g%+4.1f%+4.1f%+4.1f",
                 (double)d->gx, (double)d->gy, (double)d->gz);
    }
    row_put(3, line, (d->calib_state == 1) ? GRAY : WHITE, BLACK);

    /* ROW 4 磁力计 (没磁力计显示 "no mag") */
    if (d->who_mag == AK8963_WHO_AM_I_VAL) {
        snprintf(line, sizeof(line), "m%+4.1f%+4.1f%+4.1f",
                 (double)d->mx, (double)d->my, (double)d->mz);
    } else {
        snprintf(line, sizeof(line), "m no mag        ");
    }
    row_put(4, line, (d->who_mag == AK8963_WHO_AM_I_VAL) ? WHITE : GRAY, BLACK);

    /* ROW 5 累计欧拉 (纯陀螺积分) */
    snprintf(line, sizeof(line), "w%+5.1f%+5.1f%5.1f",
             (double)d->euler_x, (double)d->euler_y, (double)d->euler_z);
    row_put(5, line, YELLOW, BLACK);

    /* ROW 6 YPR 标签 / ROW 7 姿态 */
    row_put(6, "Y     P    R   ", WHITE, BLACK);
    snprintf(line, sizeof(line), "%+5.1f%+5.1f%+5.1f",
             (double)d->yaw, (double)d->pitch, (double)d->roll);
    row_put(7, line, (d->who_mag == AK8963_WHO_AM_I_VAL) ? WHITE : GRAY, BLACK);

    /* ROW 8 温度 + 校准状态 */
    snprintf(line, sizeof(line), "T:%+5.1fC CAL=%1u",
             (double)d->temp_c, (unsigned)d->calib_state);
    row_put(8, line, CYAN, BLACK);
}

static void MPU9250Page_OnKey(void) {
    if (key(1, down)) {
        MPU9250_ResetGyroCalib();
        UI_ForceRedraw();
    }
    /* K2..K4 当前 no-op */
}

/* ============================================================================
 * 页面注册表 (顺序 = 菜单显示顺序)
 * ============================================================================ */
const page_t pages[] = {
    { "MENU",    MenuPage_Render,    MenuPage_OnKey,    NULL },  /* [0] 菜单页 */
    { "main",    MainPage_Render,    MainPage_OnKey,    NULL },  /* [1] 主页 (上电默认进入, K5 切到菜单) */
    { "TASK1",   Task1Page_Render,   TaskPage_OnKey,    NULL },  /* [2] 题目页 1 (placeholder) */
    { "TASK2",   Task2Page_Render,   TaskPage_OnKey,    NULL },  /* [3] 题目页 2 */
    { "TASK3",   Task3Page_Render,   TaskPage_OnKey,    NULL },  /* [4] 题目页 3 */
    { "TASK4",   Task4Page_Render,   TaskPage_OnKey,    NULL },  /* [5] 题目页 4 (预留) */
    { "TASK5",   Task5Page_Render,   TaskPage_OnKey,    NULL },  /* [6] 题目页 5 (预留) */
    { "MOTOR-X", MotorXPage_Render,  MotorXPage_OnKey,  NULL },  /* [7] PD42S1 X 轴 */
    { "MOTOR-Y", MotorYPage_Render,  MotorYPage_OnKey,  NULL },  /* [8] PD42S1 Y 轴 */
    { "HUITB",   HuiduTBPage_Render, HuiduTBPage_OnKey, NULL },  /* [9] 灰度 + TB6612 */
    { "MPU9250", MPU9250Page_Render, MPU9250Page_OnKey, NULL },  /* [10] 9 轴 IMU */
};

/* ============================================================================
 * 公开 API
 * ============================================================================ */
void UI_Init(void) {
    /* g_page 启动值已在文件顶部用 MAIN_DEFAULT_PAGE 声明, 不再覆盖 */
    g_menu_sel = MAIN_BOOT_MENU_SEL;
    mark_all_rows_dirty();
}

void UI_ForceRedraw(void) {
    mark_all_rows_dirty();
}

void UI_Render(void) {
    pages[g_page].render();
    if (pages[g_page].footer_hook != NULL) {
        pages[g_page].footer_hook();
    } else {
        char line[20];
        snprintf(line, sizeof(line), "P%d/%d %s",
                 (int)(g_page + 1), (int)PAGE_COUNT, pages[g_page].name);
        row_put(9, line, WHITE, DARKBLUE);
    }
    row_flush();
}

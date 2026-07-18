/* ============================================================================
 * @file    ui.c
 * @brief   多页面 LCD 渲染 (128x160 竖屏, 8x16 字体, 行高 16 px/行, 共 10 行)
 *
 *   两层架构:
 *     - g_page = 0 → 菜单页 (MenuPage)
 *     - g_page = 1..PAGE_COUNT-1 → 详情页 (Task / 硬件)
 *
 *   菜单页功能 (用户 2026-07-15 需求):
 *     - ROW 0: 标题 "== MENU ==" (8 字符)
 *     - ROW 1..8: 8 个菜单项, 选中项加 '>' 前缀, 其它项前导空格
 *     - ROW 9: 页脚 "P1/9 MENU"
 *     - K1 down: 选中项上移 (循环: 1→8→7→...→1)
 *     - K2 down: 选中项下移 (循环: 1→2→...→8→1)
 *     - K5 down: 进入选中项 (g_page = g_menu_sel)
 *
 *   详情页统一行为:
 *     - K5 down → 返回菜单页 (g_page = 0) + 切页副作用: SM_Stop + TB6612_Stop×2
 *     - K1~K4 → pages[g_page].on_key() (空函数 = 不响应)
 *
 * ============================================================================
 * 调用方法 (主循环用法)
 * ============================================================================
 *
 *   上电 (main() 启动序列里调一次):
 *     UI_Init();                         // g_page=0 (菜单), g_menu_sel=1 (Task1), 首帧 dirty
 *
 *   主循环每帧调一次:
 *     UI_Render();                       // 当前页 render + 页脚 + 按内容 hash 刷 LCD
 *
 *   切页 / 按键触发瞬时刷新:
 *     UI_ForceRedraw();                  // 下一帧整页重画 (绕过 hash 缓存)
 *
 *   K5 路由完全由 main.c 处理, 不在本文件:
 *     - 菜单页 (g_page=0) K5 down → g_page = g_menu_sel; UI_ForceRedraw
 *     - 详情页 (g_page>0) K5 down → SM_Stop + TB6612_Stop×2 + g_page = 0; UI_ForceRedraw
 *     - 菜单页 K1/K2 → g_menu_sel 推进; UI_ForceRedraw
 *
 * ============================================================================
 *   页面注册表包含 10 页: Menu + Task1..5 + Motor-X/Y + HUITB + MPU9250
 *     [0] MenuPage     菜单页 (详情页"home", 按 K5 进入)
 *     [1] Task1Page    题目页 1 (placeholder, 后续填业务)
 *     [2] Task2Page    题目页 2 (placeholder)
 *     [3] Task3Page    题目页 3 (placeholder)
 *     [4] Task4Page    题目页 4 (placeholder, 预留)
 *     [5] Task5Page    题目页 5 (placeholder, 预留)
 *     [6] MotorPage    PD42S1 闭环步进电机 (硬件页 1)
 *     [7] HuiduTBPage  灰度 + TB6612 编码电机 (硬件页 2)
 *     [8] MPU9250Page  9 轴 IMU (硬件页 3)
 *
 *   UI_Render 每帧:
 *     1. 调 pages[g_page].render() 画该页内容 (ROW 0..8)
 *     2. 页脚 (ROW 9, y=144): footer_hook 非空用之, 否则用 "P<n>/<N> <name>"
 *
 *   LCD 总高 160, 字体高 16 (LCD_8X16) → 严格 10 行 (无行间距)
 *   y = n * 16, n = 0..9, 末行 y = 144 高 16 → 恰好填到 y = 160
 *
 * ============================================================================
 */
#include "user/UI/ui.h"
#include "Hardware/PD42S1/stepmotor.h"
#include "Hardware/PD42S1/pd42s1.h"
#include "Hardware/KEY/key.h"
#include "Hardware/Huidu/huidu.h"
#include "Hardware/TB6612/tb6612.h"
#include "Hardware/Encoder/encoder.h"
#include "Hardware/MPU9250/mpu9250.h"
#include "LCD.h"
#include "system/clock.h"
#include <stdio.h>
#include <string.h>

/* ============================================================================
 * 全局状态
 * ============================================================================ */
uint8_t       g_page     = 0;    /* 0 = 菜单页, 1..PAGE_COUNT-1 = 详情页 (上电默认菜单) */
uint8_t       g_menu_sel = 1;    /* 菜单页选中项 (1..MENU_ITEM_COUNT, 上电默认 Task1) */
const uint8_t PAGE_COUNT = 10;   /* Menu + 5 Task + X/Y 步进 + HUITB + MPU9250 */

/* ============================================================================
 * 行级脏位渲染 (按内容 hash 比对, 只画变化行)
 *
 * 设计要点:
 *   - 每行缓存: last_text (≤16 字符) + last_hash (uint16 FNV-1a)
 *   - MotorPage_Render 拼字符串 → 调 row_put → hash 变才进 dirty
 *   - UI_Render 末尾 row_flush() 把 dirty-row 提交给 LCD, 无 dirty → 零开销
 *   - 页脚 (第 10 行 y=144) 同走脏位, 切页时整页 dirty
 *
 * 哈希: 16-bit FNV-1a (一行 ≤16 字符, 实际零碰撞)
 * ============================================================================ */
#define LCD_ROWS       10    /* y=0..9 共 10 行 (最后一行 y=144 是页脚) */
#define LCD_ROW_CHARS 16    /* 一行最多 16 字符 */
typedef struct {
    char        text[LCD_ROW_CHARS + 2];
    uint16_t    hash;
    uint16_t    fg;
    uint16_t    bg;
    uint8_t     dirty;
} row_buf_t;

static row_buf_t s_rows[LCD_ROWS];
static uint8_t  s_row_count = 0;

static void row_put(uint8_t idx, const char *txt, uint16_t fg, uint16_t bg) {
    if (idx >= LCD_ROWS) return;
    row_buf_t *r = &s_rows[idx];

    /* 严格截断到 LCD_ROW_CHARS (16) 字符:
     *   - 一行最多 16 字符 (128 px / 8 px = 16)
     *   - LCD_ShowString 写完 \0 就停, 但超出 16 的部分会被画到下一行覆盖前面的字符
     *   - 绝对不在这里补空格, 由调用方自己用 snprintf 的宽度限定符 ("%.16s") 控好 */
    char buf[LCD_ROW_CHARS + 2];
    uint8_t i = 0;
    for (; i < LCD_ROW_CHARS && txt[i] != '\0'; i++) {
        buf[i] = txt[i];
    }
    buf[i] = '\0';

    uint16_t h = 21661u;
    for (const char *p = buf; *p; p++) {
        h = (uint16_t)((h ^ (uint8_t)*p) * 16777u);
    }
    if (!r->dirty && r->hash == h && strcmp(r->text, buf) == 0) {
        if (idx + 1U > s_row_count) s_row_count = (uint8_t)(idx + 1U);
        return;
    }
    /* 已确保 buf 末尾 \0, 直接 memcpy + 强补 \0 (避免 strncpy 截断警告) */
    memcpy(r->text, buf, i + 1u);
    r->hash  = h;
    r->fg    = fg;
    r->bg    = bg;
    r->dirty = 1;
    if (idx + 1U > s_row_count) s_row_count = (uint8_t)(idx + 1U);
}

/* row_flush: 把本帧所有 dirty 行写到 LCD (其实只画变化行)
 *   y 起点 = i * 16 (16 px/行, 无行间距, 10 行填满 160 px)
 *   每行内部: LCD_Fill(0,y,LCD_W,y+16,bg) + LCD_ShowString → 1 行正好 16 px 高 */
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

/* mark_all_dirty: 切页/强制刷新时调, 本帧强制重画所有行 */
static void mark_all_rows_dirty(void) {
    for (uint8_t i = 0; i < LCD_ROWS; i++) {
        s_rows[i].dirty = 1;
        s_rows[i].text[0] = '\0';
        s_rows[i].hash = 0;
    }
    s_row_count = 0;
}

/* ============================================================================
 * 菜单项显示文本表 (顺序对应 pages[1..MENU_ITEM_COUNT])
 *   - 上电默认 g_menu_sel=1 → 第 1 项 (Task1)
 *   - 显示格式: "  TASK1    " (无选中) / ">TASK1    " (选中)
 *   - 每项 ≤ LCD_ROW_CHARS=16 字符, 名称统一对齐到 8 字符宽度方便后续补字段
 *
 *   注意: pages[1..8] 的 name 字段也是这套字符串, 跟菜单项同名 (用于详情页页脚)
 * ============================================================================ */
static const char * const kMenuNames[MENU_ITEM_COUNT] = {
    "TASK1",
    "TASK2",
    "TASK3",
    "TASK4",
    "TASK5",
    "MOTOR-X",
    "MOTOR-Y",
    "HUITB",
    "MPU9250",
};

/* ============================================================================
 * 菜单页 (g_page=0)
 *
 *   布局 (16 字符/行硬约束):
 *     ROW 0   "== MENU =="   标题 (8 字符, 居中)
 *     ROW 1   "  TASK1    "   8 项菜单 (选中项加 '>', 其它前导 2 空格)
 *     ...
 *     ROW 8   "  MPU9250  "
 *     ROW 9   "P1/9 MENU"   页脚 (走缺省, name="MENU")
 *
 *   菜单项排版 (用户 2026-07-15 决定):
 *     - 16 字符 = 1 字符前缀 ('>' 或 ' ') + 8 字符名 + 7 字符留白
 *     - 选中项用 YELLOW + 黑底, 其它项用 WHITE + 黑底 (选中状态靠前缀 + 颜色)
 *     - ROW 0 用 WHITE + DARKBLUE 标题色, 跟页脚风格呼应
 *
 *   按键 (main.c 处理):
 *     K1 down → g_menu_sel-- (循环, 1↔8)
 *     K2 down → g_menu_sel++ (循环, 1↔8)
 *     K5 down → g_page = g_menu_sel (进入选中详情页)
 *
 *   为什么把 K1/K2/K5 路由放 main.c 而不是 on_key: 详情页 K5 是"返回菜单", 菜单页 K5 是"进入选中",
 *   这两件事高度耦合到 g_page 切换, main.c 集中处理更清楚 (跟原架构一致: K5 一直由 main.c 处理)
 * ============================================================================ */
static void MenuPage_Render(void) {
    char line[LCD_ROW_CHARS + 2];

    /* ROW 0: 标题 (8 字符, 居中留 4 空格) */
    row_put(0, "   == MENU ==   ", WHITE, DARKBLUE);

    /* LCD 只有 8 行菜单空间；第 9 项选中时窗口下移一行显示 2..9。 */
    uint8_t first = (g_menu_sel > 8U) ? (uint8_t)(g_menu_sel - 7U) : 1U;
    for (uint8_t row = 0U; row < 8U; row++) {
        uint8_t sel = (uint8_t)(first + row);
        snprintf(line, sizeof(line), "%c%-15.15s",
                 (g_menu_sel == sel) ? '>' : ' ',
                 kMenuNames[sel - 1U]);
        row_put((uint8_t)(1U + row), line,
                (g_menu_sel == sel) ? YELLOW : WHITE, BLACK);
    }
    /* ROW 9 = 页脚, 走 footer_hook 或缺省; 本页 footer_hook = NULL → 自动 "P1/9 MENU" */
}

static void MenuPage_OnKey(void) {
    /* 菜单页 K1/K2/K5 路由全部在 main.c 处理 (跟 g_page 切换耦合)
     * 这里留空, 防止误派发. 即: 菜单页 on_key 是 no-op */
}

/* 前向声明: row_put_keys 在下面 567 行才定义, 上面 MPU9250 / Motor / HuiduTB / Task1..5
 *   共 8 个 render 都要先调用 */
static void row_put_keys(void);

/* ============================================================================
 * 题目页模板 (Task1..Task5, g_page=1..5)
 *
 *   共享布局 (用户 2026-07-15 决定: 占位, 内容后续补):
 *     ROW 0   "k:1 1 1 1 1"   键位 (跟其它详情页一致)
 *     ROW 1   "TASK<N>"       标题 (4+数字 = 5 字符, 居中 5 空格)
 *     ROW 2   "to be added"   占位提示 (11 字符)
 *     ROW 3..8 空白
 *     ROW 9   页脚 "P<n>/<N> TASK<N>"
 *
 *   按键: K1~K4 全 no-op (待题目页内容确定后再挂具体控制)
 *   后续扩展: 改 row_put 行的文本即可, 不要破坏 16 字符约束
 *
 *   为什么用 5 个相同模板: 后续可能一次性加多道题 (5 个放一起省事), 各页独立 Task{N}Page
 *   函数便于后续差异化作扩展 (比如 Task1 要显示 IMU 姿态, Task2 要显示位置, 各开各的 render)
 * ============================================================================ */
static void TaskPage_Render(uint8_t n) {
    /* ROW 0: 键位 */
    row_put_keys();

    /* ROW 1: 标题 "TASK<n>" (居中) */
    char line[LCD_ROW_CHARS + 2];
    snprintf(line, sizeof(line), "     TASK%u     ", (unsigned)n);
    row_put(1, line, YELLOW, BLACK);

    /* ROW 2: 占位提示 */
    row_put(2, "  to be added   ", GRAY, BLACK);

    /* ROW 3..8 空白 (row_put 空串命中缓存, 零开销) */
    for (uint8_t i = 3; i <= 8; i++) {
        row_put(i, "", GRAY, BLACK);
    }
    /* ROW 9 = 页脚, 走 footer_hook 或缺省 */
}

static void TaskPage_OnKey(void) {
    /* K1~K4 全 no-op, 后续题目页业务在这里挂 */
}

static void Task1Page_Render(void) { TaskPage_Render(1U); }
static void Task2Page_Render(void) { TaskPage_Render(2U); }
static void Task3Page_Render(void) { TaskPage_Render(3U); }
static void Task4Page_Render(void) { TaskPage_Render(4U); }
static void Task5Page_Render(void) { TaskPage_Render(5U); }

/* ============================================================================
 * 第四页: MPU-9250 9 轴 IMU (加速度 + 陀螺 + 磁力计, I2C0)
 *
 *   行号 (用户约定 1 开始)              内容
 *   -----                              --------
 *   ROW 0  y=  0   k:1 1 1 1 1              K1~K5 键位
 *   ROW 1  y= 16   X       Y       Z        XYZ 标签 (紧凑)
 *   ROW 2  y= 32   a+1.20+0.34-0.98        acc  XYZ [g]    (ax/ay/az 前缀 a)
 *   ROW 3  y= 48   g+1.24+4.47+0.12        gyro XYZ [dps]  (gx/gy/gz 前缀 g)
 *   ROW 4  y= 64   m+12.3 -4.5 +6.7        mag  XYZ [uT]   (mx/my/mz 前缀 m)
 *   ROW 5  y= 80   w+060.0+000.0 360.0     累计欧拉 (euler_x/y/z, 用户 05:36 反馈)
 *   ROW 6  y= 96   Y       P       R        yaw/pitch/roll 标签
 *   ROW 7  y=112   +45.3 -12.4 +5.7        姿态 YPR [deg]
 *   ROW 8  y=128   T:+27.3C  CAL=2         温度 + 校准状态
 *   ROW 9  y=144   P<n>/<N> MPU9250        页脚
 *
 *   按键语义 (用户 2026-07-14 决定):
 *     K1 down  → MPU9250_ResetGyroCalib()   重新采集陀螺零漂 (板子静止后按)
 *     K2..K4   → no-op (后续预留 hard-iron / yaw reset)
 *     K5       → main.c 切页
 *
 *   16 字符硬约束 + 间距 1 行 (= 16 px, 字符 8 px):
 *     "X       Y       Z"      → 16 字符 (标签之间空 5 字符)
 *     "a%+4.1f%+4.1f%+4.1f"   → 1+15 = 16 字符 (前缀 a/g/m 各占 1 字符)
 *     "%+5.1f%+5.1f%+5.1f"    → 15 字符 (YPR 紧凑)
 *     "T:+27.3C  %02X"        → 11 字符 (温度 + WHO)
 *
 *   历史 (04:33 用户反馈: "之间空多了, 现在还空着 5 个"):
 *     - 04:14 → 标签-数据贴紧, 间距 1 行 ✓ (用户认定的最终版本)
 *     - 04:33 (前一版) → 我擅自改成 4 对标签-数据各间隔 2 行, 用户否决
 *
 *   04:33 实测数据 (RAW 12 秒静止) → hard-iron 已填实:
 *     bias_gx/gy/gz = +1.32, +4.86, -0.78 dps (校准后残差 ±0.04 dps, 5 秒 yaw 漂 0.2°)
 *     hard_iron = (+2.71, -36.5, -35.3) uT (yaw 漂移从 ±1.2° → ±0.5°)
 * ============================================================================ */
static void MPU9250Page_Render(void) {
    char line[LCD_ROW_CHARS + 2];
    const MPU9250_Data *d = MPU9250_GetData();

    /* ROW 0: 键位 */
    row_put_keys();

    if (!d->ok) {
        row_put(1, "X   Y   Z   ", GRAY, BLACK);
        row_put(2, "a no dev          ", GRAY, BLACK);
        row_put(3, "g no dev          ", GRAY, BLACK);
        row_put(4, "m no dev          ", GRAY, BLACK);
        row_put(5, "w no dev          ", GRAY, BLACK);
        row_put(6, "Y   P   R   ", GRAY, BLACK);
        row_put(7, "y no dev          ", GRAY, BLACK);
        row_put(8, "T:----C  --       ", GRAY, BLACK);
        return;
    }

    /* ROW 1: XYZ 标签 */
    row_put(1, "X     Y    Z   ", WHITE, BLACK);

    /* ROW 2: 加速度 XYZ [g] — 前缀 a */
    snprintf(line, sizeof(line), "a%+4.1f%+4.1f%+4.1f",
             (double)d->ax, (double)d->ay, (double)d->az);
    row_put(2, line, WHITE, BLACK);

    /* ROW 3: 陀螺 XYZ [dps] — 前缀 g
     *   逻辑: calib_state != 1 (即 0=IDLE 或 2=OK) → 显示数值 (LCD 始终显示)
     *         calib_state == 1 (CAL_RUNNING 校准中, 200 帧未满) → 显示 "cal..."
     *   (用户 05:02 反馈: "让一直显示", 即只在短暂校准中切到 cal..., 其他时候正常显示)
     */
    if (d->calib_state == 1) {
        snprintf(line, sizeof(line), "g cal...        ");
    } else {
        snprintf(line, sizeof(line), "g%+4.1f%+4.1f%+4.1f",
                 (double)d->gx, (double)d->gy, (double)d->gz);
    }
    row_put(3, line, (d->calib_state == 1) ? GRAY : WHITE, BLACK);

    /* ROW 4: 磁力计 XYZ [uT] — 前缀 m */
    if (d->who_mag == AK8963_WHO_AM_I_VAL) {
        snprintf(line, sizeof(line), "m%+4.1f%+4.1f%+4.1f",
                 (double)d->mx, (double)d->my, (double)d->mz);
    } else {
        snprintf(line, sizeof(line), "m no mag        ");
    }
    row_put(4, line, (d->who_mag == AK8963_WHO_AM_I_VAL) ? WHITE : GRAY, BLACK);

    /* ROW 5: 累计欧拉角 [°] — 前缀 w
     *   用户 05:36 反馈: "m 下面加一行 w, 显示方向 (0-360 度), 上电时为 0, 移动后清晰看到动了多少"
     *   格式 "w+060.0+000.0 360.0" → 16 字符 (用户原话: "w360.0 360.0 360.0")
     *   显示 euler_x (roll) / euler_y (pitch) / euler_z (yaw):
     *     - euler_x: 绕 X 轴累计转角 (左翻右翻)  → [-180,+180]
     *     - euler_y: 绕 Y 轴累计转角 (上翘下俯)  → [-180,+180]
     *     - euler_z: 绕 Z 轴累计转角 (左右转)    → [  0,+360), 右转为正
     *   注意: 纯陀螺积分, 不抗漂, 长时间会偏 (但用户要"清晰看到移动量")
     */
    snprintf(line, sizeof(line), "w%+5.1f%+5.1f%5.1f",
             (double)d->euler_x,
             (double)d->euler_y,
             (double)d->euler_z);
    row_put(5, line, YELLOW, BLACK);

    /* ROW 6: YPR 标签 */
    row_put(6, "Y     P    R   ", WHITE, BLACK);

    /* ROW 7: yaw / pitch / roll [deg] — 绝对姿态 (磁力校准) */
    snprintf(line, sizeof(line), "%+5.1f%+5.1f%+5.1f",
             (double)d->yaw,
             (double)d->pitch,
             (double)d->roll);
    row_put(7, line, (d->who_mag == AK8963_WHO_AM_I_VAL) ? WHITE : GRAY, BLACK);

    /* ROW 8: 温度 [°C] + 校准状态 (从原 ROW 7 下移)
     *   "T:+27.3C  CAL=2" → 16 字符
     *   calib_state 含义: 0=IDLE 未开始  1=RUNNING 校准中  2=OK 完成
     */
    snprintf(line, sizeof(line), "T:%+5.1fC CAL=%1u",
             (double)d->temp_c, (unsigned)d->calib_state);
    row_put(8, line, CYAN, BLACK);

    /* ROW 9 = 页脚, 走 footer_hook 或缺省; 本页 footer_hook = NULL */
}

static void MPU9250Page_OnKey(void) {
    /* K1: 重置陀螺零漂 (用户 2026-07-14 反馈: 板子没静止 / bias 偏了时手动重置)
     *   提示: 重置后 1 秒内板子必须静止, 否则新 bias 仍偏
     */
    if (key(1, down)) {
        MPU9250_ResetGyroCalib();
        UI_ForceRedraw();
    }
    /* K2..K4 当前 no-op, 后续如果加 hard-iron 校准 / yaw reset 在这里挂 */
    /* K5 由 main.c 处理切页 */
}

/* ============================================================================
 * PD42S1 X/Y 轴硬件调试页
 * ============================================================================ */
#define MOTOR_PULSES_PER_REV  51200L

/* 移动步长档 (用户 2026-07-15 决定: 短按 K4 在 1°/3°/5°/10°/0.1° 间循环) */
static const uint16_t kMotorStepTenths[] = { 10U, 30U, 50U, 100U, 1U };
static uint8_t s_motor_step_index[2] = { 0U, 0U };

/* 移动速度档 (RPM, 2026-07-15 决定: 长按 K2 循环切换, 默认 30 = 中等)
 *   现有默认值 60/100 用户反馈"太快", 给 4 档, 默认取中间偏慢:
 *     15 RPM  → 几乎爬行, 极限慢
 *     30 RPM  → 慢, 适合手动调试 (默认)
 *     60 RPM  → 中等 (旧默认值)
 *     120 RPM → 快, 极限快
 *   范围约束: PD42S1 协议 uint8 speed (限到 6000), 4 档都远低于上限 */
static const uint16_t kMotorSpeedRpm[] = { 10U, 20U, 40U, 60U };
static uint8_t s_motor_speed_index[2] = { 1U, 1U };   /* 默认 index=1 → 30 RPM (中等) */

/* 移动加速度档 (accel, 0~200, 2026-07-15 决定: 长按 K1 循环切换, 默认 50 = 中等)
 *   给 3 档, 默认取中间:
 *     20  → 慢加减速, 适合慢速精细定位
 *     50  → 中等, 平衡速度与冲击 (默认)
 *     100 → 快, 旧默认值, 用于快速大行程
 *   accel=0 是协议允许的"直接启动", 不用 */
static const uint8_t kMotorAccel[] = { 15U, 30U, 50U };
static uint8_t s_motor_accel_index[2] = { 1U, 1U };    /* 默认 index=1 → 50 (中等) */

/* 度 → 脉冲 (有符号, 四舍五入; 一圈 51200 脉冲 = 360°)
 * 范围: int32 ±2^31 ≈ ±42000 圈 (远超电机行程, 不会溢出) */
#define MOTOR_DEG_TO_PULSES(deg)  \
    ((int32_t)(((int64_t)(deg) * (int64_t)MOTOR_PULSES_PER_REV + 180LL) / 360LL))

/* SM_zeroset 用到的左右限位参数 (用户 2026-07-15 决定, 硬件固定不变)
 *   X 轴:  0 .. +1800°  (转 5 圈)
 *   Y 轴: -90 .. +90°  (短行程, 不对称) */
#define MOTOR_X_LEFT_DEG    0
#define MOTOR_X_RIGHT_DEG   1800
#define MOTOR_Y_LEFT_DEG    (-50)
#define MOTOR_Y_RIGHT_DEG   80
#define MOTOR_ZEROSET_TIMEOUT_MS  10000U   /* 见 stepmotor.h 注释, 10000~30000 推荐 */
#define MOTOR_ZEROSET_AUTO_HOME   true     /* 驱动器下次上电自动回零 */
#define MOTOR_ZEROSET_LIMIT_ON    true     /* 开启左右限位 (行程框住) */

/* K4 长按 → 一次性把 X/Y 轴的左/右原点坐标 + 超时 + 上电自动回零 + 限位开关 6 帧写入
 *   stepmotor.c 节流器 10ms 间隔串行发, 约 60ms 写完 */
static void MotorPage_ZerosetDefaults(sm_motor_t motor) {
    if (motor == SM_X) {
        SM_zeroset(SM_X,
                   MOTOR_DEG_TO_PULSES(MOTOR_X_LEFT_DEG),
                   MOTOR_DEG_TO_PULSES(MOTOR_X_RIGHT_DEG),
                   MOTOR_ZEROSET_TIMEOUT_MS,
                   MOTOR_ZEROSET_AUTO_HOME,  MOTOR_ZEROSET_LIMIT_ON);
    } else {
        SM_zeroset(SM_Y,
                   MOTOR_DEG_TO_PULSES(MOTOR_Y_LEFT_DEG),
                   MOTOR_DEG_TO_PULSES(MOTOR_Y_RIGHT_DEG),
                   MOTOR_ZEROSET_TIMEOUT_MS,
                   MOTOR_ZEROSET_AUTO_HOME,  MOTOR_ZEROSET_LIMIT_ON);
    }
}

static uint8_t MotorAxisIndex(sm_motor_t motor) {
    return (motor == SM_Y) ? 1U : 0U;
}

static int32_t MotorStepPulses(sm_motor_t motor) {
    uint16_t tenths = kMotorStepTenths[s_motor_step_index[MotorAxisIndex(motor)]];
    return (int32_t)(((uint32_t)tenths * (uint32_t)MOTOR_PULSES_PER_REV + 1800U) / 3600U);
}

static void MotorPage_PutHexRows(uint8_t first_row, char prefix,
                                 const volatile uint8_t *data, uint8_t len,
                                 uint16_t color) {
    char line[LCD_ROW_CHARS + 2];
    char *p;
    uint8_t first = (len > 7U) ? 7U : len;

    if (data == NULL || len == 0U) {
        snprintf(line, sizeof(line), "%c:------------", prefix);
        row_put(first_row, line, GRAY, BLACK);
        row_put((uint8_t)(first_row + 1U), "", GRAY, BLACK);
        return;
    }

    snprintf(line, sizeof(line), "%c:", prefix);
    p = line + 2;
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

static void MotorPage_Render(sm_motor_t motor) {
    char line[LCD_ROW_CHARS + 2];
    uint8_t axis = MotorAxisIndex(motor);
    uint16_t tenths = kMotorStepTenths[s_motor_step_index[axis]];
    int32_t pos = SM_GetPosition(motor);
    double degree = ((double)pos * 360.0) / (double)MOTOR_PULSES_PER_REV;
    uint8_t tx_len = 0U;
    bool tx_fired = false;
    const volatile uint8_t *tx = PD42S1_GetTxBuffer((uint8_t)motor, &tx_len, &tx_fired);
    pd42_frame_t *rx = PD42S1_GetFrameFor((uint8_t)motor);
    uint8_t rx_raw[10];
    uint8_t rx_len = 0U;

    row_put_keys();

    snprintf(line, sizeof(line), "AX:%c STEP:%4.1f",
             (motor == SM_X) ? 'X' : 'Y', (double)tenths / 10.0);
    row_put(1, line, YELLOW, BLACK);

    snprintf(line, sizeof(line), "POS:%+11ld", (long)pos);
    row_put(2, line, WHITE, BLACK);
    snprintf(line, sizeof(line), "DEG:%+12.2f", degree);
    row_put(3, line, GREEN, BLACK);

    MotorPage_PutHexRows(4U, 'T', tx, tx_fired ? tx_len : 0U, CYAN);
    /* debug: ROW 5 TXA:X/Y ADDR:xx 已废弃, 用户 04:19 反馈不影响显示 — 删除 */
    row_put(5, "", WHITE, BLACK);

    if (rx->function_code != 0U) {
        rx_raw[rx_len++] = PD42S1_FRAME_HEAD;
        rx_raw[rx_len++] = rx->slave_addr;
        rx_raw[rx_len++] = rx->function_code;
        for (uint8_t i = 0U; i < rx->data_len && rx_len < 8U; i++) {
            rx_raw[rx_len++] = rx->data[i];
        }
        rx_raw[rx_len++] = (uint8_t)rx->checksum;
        rx_raw[rx_len++] = PD42S1_FRAME_TAIL;
    }
    MotorPage_PutHexRows(6U, 'R', rx_raw, rx_len,
                         (rx_len > 0U && rx->error_code == PD42_ACK_OK) ? GREEN : RED);

    /* ROW 8: 当前速度档 + 加速度档 (用户 2026-07-15 决定, 替换原 "K3:R/H K4:STEP")
     *   格式: "S:30RPM A:50 K3:R/H" = 16 字符 (YELLOW 当前档 + 灰色按键提示)
     *   - "S:30RPM" = 7 字符: 速度当前档 (RPM), 长按 K2 循环
     *   - " A:50"   = 5 字符: 加速度当前档,       长按 K1 循环
     *   - " K3:R/H" = 7 字符: K3 短按读位置 / 长按回零 (剩余按键提示简短化, 不再挤 K4 STEP 因为 K4 短按切步长语义用户已熟悉) */
    uint8_t s_idx = s_motor_speed_index[axis];
    uint8_t a_idx = s_motor_accel_index[axis];
    snprintf(line, sizeof(line), "S:%3uRPM A:%3u",  /* "S: 30RPM A: 50" = 13 字符 */
             (unsigned)kMotorSpeedRpm[s_idx < 4U ? s_idx : 0U],
             (unsigned)kMotorAccel[a_idx < 3U ? a_idx : 0U]);
    row_put(8, line, YELLOW, BLACK);
}

static void MotorPage_OnKey(sm_motor_t motor) {
    uint8_t axis = MotorAxisIndex(motor);
    uint16_t speed_rpm = kMotorSpeedRpm[s_motor_speed_index[axis] < 4U
                                          ? s_motor_speed_index[axis] : 0U];
    uint8_t  accel     = kMotorAccel[s_motor_accel_index[axis] < 3U
                                       ? s_motor_accel_index[axis] : 0U];

    /* K1 down: 正转 (用户 2026-07-15 决定保留原语义);
     * K1 long_press: 切加速度档 (3 档: 20/50/100, 默认 50 = 中等)
     *   长按不影响 down: down 在按下沿触发, long_press 在按住超时后触发,
     *   二者不会冲突. */
    if (key(1, down)) {
        SM_Move(motor, R, accel, speed_rpm, MotorStepPulses(motor));
        UI_ForceRedraw();
    }
    if (key(1, long_press)) {
        s_motor_accel_index[axis] = (uint8_t)((s_motor_accel_index[axis] + 1U) % 3U);
        UI_ForceRedraw();
    }
    /* K2 down: 反转 (用户 2026-07-15 决定保留原语义);
     * K2 long_press: 切速度档 (4 档: 15/30/60/120 RPM, 默认 30 = 中等)
     *   长按不影响 down: 同 K1. */
    if (key(2, down)) {
        SM_Move(motor, L, accel, speed_rpm, MotorStepPulses(motor));
        UI_ForceRedraw();
    }
    if (key(2, long_press)) {
        s_motor_speed_index[axis] = (uint8_t)((s_motor_speed_index[axis] + 1U) % 4U);
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
    /* K4 短按: 松开时切精度档 (不能用 down, 否则 K4 长按会先发 down 把档切了)
     * K4 长按: 6 帧配置序列 - 写左/右原点 + 超时 + 上电自动回零 + 限位开关 + 落盘
     *          左右限位值由硬件常量决定 (X: 0..1800°, Y: -105..75°) */
    if (key(4, up)) {
        s_motor_step_index[axis] = (uint8_t)((s_motor_step_index[axis] + 1U) % 5U);
        UI_ForceRedraw();
    }
    if (key(4, long_press)) {
        MotorPage_ZerosetDefaults(motor);
        UI_ForceRedraw();
    }
}

static void MotorXPage_Render(void) { MotorPage_Render(SM_X); }
static void MotorYPage_Render(void) { MotorPage_Render(SM_Y); }
static void MotorXPage_OnKey(void) { MotorPage_OnKey(SM_X); }
static void MotorYPage_OnKey(void) { MotorPage_OnKey(SM_Y); }

/* ============================================================================
 * 第三页: 灰度 + TB6612 编码电机混和页 (用户 04:06 反馈修订)
 *
 *   行号 (用户约定: 从 1 开始)              内容
 *   -----                                  --------
 *   ROW 0  (第 1 行)  y=  0   k:1 1 1 1 1                K1~K5 键位 (0=按下, 1=松开)
 *   ROW 1  (第 2 行)  y= 16   8 路灰度二值化 (按 K1 模式切: MUX_ADC 8 位 / FIRST5_GPIO 5 位 /
 *                                  ALL8_GPIO 8 位 / 模式名缩写)
 *   ROW 2  (第 3 行)  y= 32   CH1..CH2 ADC 原始值 (MUX_ADC 模式)        / GPIO 模式: 空
 *   ROW 3  (第 4 行)  y= 48   CH3..CH4 ADC 原始值 (MUX_ADC 模式)        / GPIO 模式: 空
 *   ROW 4  (第 5 行)  y= 64   CH5..CH6 ADC 原始值 (MUX_ADC 模式)        / GPIO 模式: 空
 *   ROW 5  (第 6 行)  y= 80   CH7..CH8 ADC 原始值 (MUX_ADC 模式)        / GPIO 模式: 空
 *   ROW 6  (第 7 行)  y= 96   L:+12345                     左电机编码器值 (独立一行)
 *   ROW 7  (第 8 行)  y=112   R:+12345                     右电机编码器值 (独立一行)
 *   ROW 8  (第 9 行)  y=128   SEL:>L 60 R 40              当前 selected + 两路 PWM 档
 *   ROW 9  (第 10 行) y=144   P<n>/<N> <name> (页脚)       末行显示页数
 *
 *   按键语义 (本页专用, 用户 04:06 反馈):
 *     K1 down  → Huidu_SetMode(Next(MUX_ADC → FIRST5_GPIO → ALL8_GPIO → MUX_ADC))
 *                                 轮转 3 种灰度模式 (用户 04:06 新增)
 *     K2 down  → TB6612_Run(L, FWD) + TB6612_Run(R, FWD)   两电机同时正转
 *     K2 up    → TB6612_Stop(L)  + TB6612_Stop(R)
 *     K3 down  → TB6612_Run(L, REV) + TB6612_Run(R, REV)
 *     K3 up    → TB6612_Stop(L)  + TB6612_Stop(R)
 *     K4 短按  → TB6612_NextLevel(selected)                PWM 档 (0=60% → 1=40% → 2=20%)
 *     K4 长按  → TB6612_ToggleSelected()                   selected: L ↔ R
 *     K5       → main.c 切页 (全局)
 *
 *   编码器方向 + TB6612 映射 (用户 04:06 反馈):
 *     R 电机 (AIN1=PB6/AIN2=PB7): 物理接线决定按 K2 后电机方向反, 需翻 IN (set_in R 路调换)
 *     L 电机 (BIN1=PB23/BIN2=PB27): 当前方向对, 但编码值符号反 → encoder.c 调整查表符号
 *     PWM 等级: lv0=60% (最快) / lv1=40% / lv2=20% (最慢), 翻转 kLevelCompare 表
 * ============================================================================ */
static void row_put_keys(void) {
    char line[LCD_ROW_CHARS + 2];
    snprintf(line, sizeof(line), "k:%d %d %d %d %d",
             key_pressed(1) ? 0 : 1,
             key_pressed(2) ? 0 : 1,
             key_pressed(3) ? 0 : 1,
             key_pressed(4) ? 0 : 1,
             key_pressed(5) ? 0 : 1);
    row_put(0, line, WHITE, BLACK);
}

/* ROW 1 按模式画二值化: MUX_ADC 8 位 / FIRST5_GPIO 5 位 / ALL8_GPIO 8 位
 *   只画数据位 + 空格分隔, 不附加模式名后缀 (用户 04:35 反馈: 5G 字符含义不明)
 */
static void row_put_huidu_digital(char *line, uint8_t digital, huidu_mode_t mode) {
    char *p = line;
    if (mode == HUIDU_MODE_FIRST5_GPIO) {
        /* 5 位: "1 1 1 1 1" = 9 字符, 后面 7 字符留空 (mode 名不放 ROW 1) */
        for (uint8_t i = 0; i < 5; i++) {
            uint8_t bit = (uint8_t)(1U << i);
            *p++ = (digital & bit) ? '1' : '0';
            if (i < 4U) *p++ = ' ';
        }
    } else {
        /* 8 位 (MUX_ADC 或 ALL8_GPIO 都用相同格式): "1 1 1 1 1 1 1 1" = 15 字符 */
        for (uint8_t i = 0; i < 8; i++) {
            uint8_t bit = (uint8_t)(1U << i);
            *p++ = (digital & bit) ? '1' : '0';
            if (i < 7U) *p++ = ' ';
        }
    }
    *p = '\0';
}

static void HuiduTBPage_Render(void) {
    char line[LCD_ROW_CHARS + 2];

    /* ROW 0: 键位 */
    row_put_keys();

    /* 取灰度快照 + 当前模式 (K1 轮转) */
    uint8_t  digital = g_huidu_sensor.Digital;
    huidu_mode_t mode = Huidu_GetMode();

    /* ROW 1: 二值化 (按模式) */
    row_put_huidu_digital(line, digital, mode);
    row_put(1, line, WHITE, BLACK);

    /* ROW 2..5: ADC 原始值 (CH1..CH8, 每行两路) — 只在 MUX_ADC 模式有效, GPIO 模式时空 */
    if (mode == HUIDU_MODE_MUX_ADC) {
        uint16_t raw[8];
        Huidu_GetAnalog(raw);
        static const uint8_t kPairs[4][2] = {
            {0, 1}, {2, 3}, {4, 5}, {6, 7}
        };
        for (uint8_t row = 0; row < 4; row++) {
            uint8_t a = kPairs[row][0];
            uint8_t b = kPairs[row][1];
            uint16_t ra = (raw[a] > 4095U) ? 4095U : raw[a];
            uint16_t rb = (raw[b] > 4095U) ? 4095U : raw[b];
            snprintf(line, sizeof(line), "%u:%04u %u:%04u",
                     (unsigned)(a + 1U), (unsigned)ra,
                     (unsigned)(b + 1U), (unsigned)rb);
            uint16_t vmax = (ra > rb) ? ra : rb;
            uint16_t fg = (vmax > 2500U) ? GREEN :
                          (vmax < 1500U) ? RED : WHITE;
            row_put((uint8_t)(2 + row), line, fg, BLACK);
        }
    } else {
        for (uint8_t row = 2; row <= 5; row++) {
            row_put(row, "", GRAY, BLACK);
        }
    }

    /* ROW 6 (第 7 行): L 编码器值 (独立一行, 用户 04:06)
     *   "L:+12345" = 8 字符 (前缀 + 有符号 5 位), 留 8 字符空
     */
    int32_t cL = Encoder_GetCountL();
    snprintf(line, sizeof(line), "L:%+05ld        ", (long)cL);
    row_put(6, line, WHITE, BLACK);

    /* ROW 7 (第 8 行): R 编码器值 (独立一行, 用户 04:06) */
    int32_t cR = Encoder_GetCountR();
    snprintf(line, sizeof(line), "R:%+05ld        ", (long)cR);
    row_put(7, line, WHITE, BLACK);

    /* ROW 8 (第 9 行): SEL + 两路 PWM 档
     *   "SEL:>L 60 R 40" = 14 字符, 选中电机加 '>'
     */
    tb_motor_t sel = TB6612_GetSelected();
    uint8_t lL = TB6612_GetLevel(TB_MOTOR_L);
    uint8_t lR = TB6612_GetLevel(TB_MOTOR_R);
    static const uint8_t kLvPct[3] = { 20, 40, 60 };  /* 跟 tb6612.c kLevelCompare 同步正向 */
    snprintf(line, sizeof(line), "SEL:%sL %u R %u",
             (sel == TB_MOTOR_L) ? ">" : " ",
             (unsigned)kLvPct[lL < 3U ? lL : 0U],
             (unsigned)kLvPct[lR < 3U ? lR : 0U]);
    row_put(8, line, WHITE, BLACK);
    /* ROW 9 = 页脚, 走 footer_hook 或缺省; 本页 footer_hook = NULL */
}

static void HuiduTBPage_OnKey(void) {
    /* K1 短按: 轮转 3 种灰度模式 (用户 04:06 反馈)
     *   MUX_ADC  → FIRST5_GPIO → ALL8_GPIO → MUX_ADC ...
     */
    if (key(1, down)) {
        Huidu_NextMode();
        UI_ForceRedraw();
    }

    /* K2 down/up: 两电机正转 / 停 */
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
    /* K3 down/up: 两电机反转 / 停 */
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
    /* K4 短按: 切 PWM 档 (作用于 selected 电机) */
    if (key(4, down)) {
        TB6612_NextLevel(TB6612_GetSelected());
        UI_ForceRedraw();
    }
    /* K4 长按: 切 selected */
    if (key(4, long_press)) {
        TB6612_ToggleSelected();
        UI_ForceRedraw();
    }
    /* K5 由 main.c 处理切页 */
}

/* ============================================================================
 * 页面注册表 (编译期常量, 顺序 = 菜单显示顺序 + 页脚名)
 *   PAGE_COUNT=10: [0] Menu + [1..5] Task1..5 + [6..9] 4 个硬件页
 *
 *   菜单项 kMenuNames[i] 对应 pages[i+1] 的 name, 菜单页 g_page=0 用 kMenuNames[] 自己渲染
 *   详情页页脚用 pages[g_page].name 渲染 → "P<n>/<N> <name>"
 *
 *   注意: pages[1..5] 的 name 用字符串字面量, 与 kMenuNames 同步, 但**不依赖** kMenuNames
 *   (kMenuNames 只在 MenuPage 用, 详情页不查表 → 修改 kMenuNames 不影响详情页)
 * ============================================================================ */
const page_t pages[] = {
    { "MENU",    MenuPage_Render,    MenuPage_OnKey,    NULL },                       /* [0] 菜单页 */
    { "TASK1",   Task1Page_Render,   TaskPage_OnKey,    NULL },                       /* [1] 题目页 1 (placeholder) */
    { "TASK2",   Task2Page_Render,   TaskPage_OnKey,    NULL },                       /* [2] 题目页 2 (placeholder) */
    { "TASK3",   Task3Page_Render,   TaskPage_OnKey,    NULL },                       /* [3] 题目页 3 (placeholder) */
    { "TASK4",   Task4Page_Render,   TaskPage_OnKey,    NULL },                       /* [4] 题目页 4 (placeholder, 预留) */
    { "TASK5",   Task5Page_Render,   TaskPage_OnKey,    NULL },                       /* [5] 题目页 5 (placeholder, 预留) */
    { "MOTOR-X", MotorXPage_Render,  MotorXPage_OnKey,  NULL },                       /* [6] 硬件: PD42S1 X 轴 */
    { "MOTOR-Y", MotorYPage_Render,  MotorYPage_OnKey,  NULL },                       /* [7] 硬件: PD42S1 Y 轴 */
    { "HUITB",   HuiduTBPage_Render, HuiduTBPage_OnKey, NULL },                       /* [8] 硬件: 灰度 + TB6612 */
    { "MPU9250", MPU9250Page_Render, MPU9250Page_OnKey, NULL },                       /* [9] 硬件: 9 轴 IMU */
};

void UI_Init(void) {
    g_page     = 0;       /* 上电进菜单页 (g_page=0) */
    g_menu_sel = 1;       /* 默认选中 Task1 */
    /* 上电后第一帧必须整页画: LCD_Init(BLUE) 把屏幕涂了蓝底,
     * 第一帧 UI_Render 把每个 row 都标 dirty, row_flush 完整覆盖整屏。 */
    mark_all_rows_dirty();
}

/* ============================================================================
 * UI 强制刷新: 切页 / 按键触发 → mark_all_rows_dirty 让下一帧整页重画;
 * 之后自动退回"按内容脏位"模式 (内容变化才真画)。
 * ============================================================================ */
void UI_ForceRedraw(void) {
    mark_all_rows_dirty();
}

void UI_Render(void) {
    /* 1. 当前页 -> 拼内容 → row_put (无变化行直接命中缓存, 零开销) */
    pages[g_page].render();

    /* 2. 页脚 (第 10 行 y=144):
     *   - footer_hook != NULL → 该页自管 (如 TB6612 在 ROW 9 塞"当前 PWM")
     *   - 否则 → 走缺省 "P<n>/<N> <name>" */
    if (pages[g_page].footer_hook != NULL) {
        pages[g_page].footer_hook();
    } else {
        char line[20];
        snprintf(line, sizeof(line), "P%d/%d %s",
                 (int)(g_page + 1), (int)PAGE_COUNT, pages[g_page].name);
        row_put(9, line, WHITE, DARKBLUE);
    }

    /* 3. 把本帧所有 dirty 行写到 LCD (变化行才真画, 静止行零操作) */
    row_flush();
}

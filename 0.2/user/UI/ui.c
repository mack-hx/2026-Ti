/* ============================================================================
 * @file    ui.c
 * @brief   多页面 LCD 渲染 (128x160 竖屏, 8x16 字体, 行高 16 px/行, 共 10 行)
 *
 * ============================================================================
 * 调用方法 (主循环用法)
 * ============================================================================
 *
 *   上电 (main() 启动序列里调一次):
 *     UI_Init();                         // g_page=0, 首帧 mark_all_rows_dirty
 *
 *   主循环每帧调一次:
 *     UI_Render();                       // 当前页 render + 页脚 + 按内容 hash 刷 LCD
 *
 *   切页 / 按键触发瞬时刷新:
 *     UI_ForceRedraw();                  // 下一帧整页重画 (绕过 hash 缓存)
 *
 *   切页副作用由 main.c 处理, 不在本文件:
 *     K5 down → SM_Stop + TB6612_Stop×2 + g_page++ + UI_ForceRedraw
 *
 * ============================================================================
 *   页面注册表 (pages[]):
 *     [0] EmptyPage     全黑, K1~K4 忽略 (空函数 on_key)
 *     [1] MotorPage     PD42S1 闭环步进电机 (ROW 0..5 内容)
 *     [2] HuiduTBPage   灰度 + TB6612 编码电机混和页 (用户 02:55 要求"和 TB6612 放同一页")
 *                       ROW 0   键位
 *                       ROW 1   8 路灰度二值化 (固定 MUX_ADC)
 *                       ROW 2..5 灰度 ADC 原始值
 *                       ROW 6   左电机编码值 (L = PB4/PB5)
 *                       ROW 7   右电机编码值 (R = PA28/PA31)
 *                       ROW 8   PWM 档 + 当前 selected
 *                       ROW 9   页脚 "P<n>/<N> HUITB"
 *
 *   UI_Render 每帧:
 *     1. 调 pages[g_page].render() 画该页内容 (ROW 0..8)
 *     2. 页脚 (ROW 9, y=144): footer_hook 非空用之, 否则用 "P<n>/<N> <name>"
 *
 *   LCD 总高 160, 字体高 16 (LCD_8X16) → 严格 10 行 (无行间距)
 *   y = n * 16, n = 0..9, 末行 y = 144 高 16 → 恰好填到 y = 160
 *
 *   行号约定 (用户 02:55): 行从 1 开始数, ROW 0..9 = "第 1..10 行"
 *   ROW 6/7/8 (= 7/8/9 行) 是空闲行, HuiduTBPage 用这三行展示 TB6612 数据
 * ============================================================================
 */
#include "user/UI/ui.h"
#include "Hardware/PD42S1/stepmotor.h"
#include "Hardware/PD42S1/pd42s1.h"
#include "Hardware/KEY/key.h"
#include "Hardware/Huidu/huidu.h"
#include "Hardware/TB6612/tb6612.h"
#include "Hardware/Encoder/encoder.h"
#include "LCD.h"
#include "system/clock.h"
#include <stdio.h>
#include <string.h>

/* ============================================================================
 * 全局状态
 * ============================================================================ */
uint8_t       g_page = 0;
const uint8_t PAGE_COUNT = 3;   /* 与 pages[] 长度同步 */

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

/* (空 operation: 切页时由 mark_all_rows_dirty 处理; 这里保留扩展点) */
static void EmptyPage_Render(void) {
    /* UI_Render 顶部已整屏 Fill 黑, 这里啥也不做 */
}

static void EmptyPage_OnKey(void) {
    /* 空页: K1~K4 全部不响应 (main.c 直接派发到此函数, 不读按键) */
}

/* ============================================================================
 * 第二页: PD42S1 闭环步进电机
 *
 *   ROW 0  y=  0  K1~K5 键位
 *   ROW 1  y= 16  POS + ERR + STATE  (一整行)
 *   ROW 2  y= 32  TX 主行 (≤7 字节 HEX)
 *   ROW 3  y= 48  TX 续行 (余下 HEX)
 *   ROW 4  y= 64  RX 主行 (HEAD ADDR FUNC ERR)
 *   ROW 5  y= 80  RX 续行
 *   ROW 9  y=144  页脚 "P<n>/<N> <name>"
 *
 * 本函数只拼内容 → row_put, 不直接画 LCD。LCD 渲染交给 UI_Render 末尾的 row_flush
 * ============================================================================ */
static void MotorPage_Render(void) {
    /* 关键约束: 一行最多 LCD_ROW_CHARS=16 字符, 不补空格, 多余字节由 row_put 截断 */
    char line[LCD_ROW_CHARS + 2];
    pd42_frame_t *f = PD42S1_GetFrame();

    /* ROW 0: 键位 (固定 14 字符: "k:1 1 1 1 1") */
    snprintf(line, sizeof(line), "k:%d %d %d %d %d",
             key_pressed(1) ? 0 : 1,
             key_pressed(2) ? 0 : 1,
             key_pressed(3) ? 0 : 1,
             key_pressed(4) ? 0 : 1,
             key_pressed(5) ? 0 : 1);
    row_put(0, line, WHITE, BLACK);

    /* ROW 1: POS / ERR / STATE
     *   紧凑无空格: "P:1234567 E:01 IDL" → "P:" + sm_pos + " E:" + err + " " + state
     *   sm_pos 最大 99999999 (实际 int32 一圈 51200 远小于此) → 至少 6 位数字 + "P:"
     *   12 字符稳, 7 位数时 13 字符, 8 位数时 14 字符 → row_put 兜底 16 */
    const char *state_str =
        (sm_state == 1) ? "FWD" :
        (sm_state == 2) ? "REV" :
        (sm_state == 3) ? "POS" : "IDL";
    snprintf(line, sizeof(line), "P:%ldE:%02X%s",
             (long)sm_pos, (unsigned)sm_err, state_str);
    row_put(1, line,
            (sm_err == 0x01 ? GREEN : RED), BLACK);

    /* ROW 2/3: TX HEX
     *   每字节 HEX = "%02X" (2 字符), 无空格;
     *   ROW 2 容纳 7 字节 = 14 字符 + "T:" 前缀 = 16 字符, 刚好一行
     *   ROW 3 余下 ≤7 字节 = ≤14 字符
     *   超过的字节由 row_put 截断 */
    if (g_tx_fired && g_tx_buffer_len >= 4u && g_tx_buffer_len <= 64u) {
        uint8_t *b = (uint8_t *)g_tx_buffer;
        uint8_t n = g_tx_buffer_len;
        uint8_t first = (n > 7u) ? 7u : n;

        /* ROW 2: "T:" + 7 字节 HEX */
        snprintf(line, sizeof(line), "T:");
        char *p = line + 2;
        for (uint8_t i = 0; i < first; i++) p += snprintf(p, 3, "%02X", b[i]);
        *p = '\0';
        row_put(2, line, CYAN, BLACK);

        /* ROW 3: 余下 HEX (无 "T:" 前缀) */
        if (n > 7u) {
            uint8_t r = (uint8_t)(n - 7u);
            if (r > 7u) r = 7u;
            p = line;
            for (uint8_t i = 0; i < r; i++) p += snprintf(p, 3, "%02X", b[7u + i]);
            *p = '\0';
            row_put(3, line, CYAN, BLACK);
        } else {
            /* 内容空 → hash 同 "" → 命中缓存, 零操作 */
            row_put(3, "", CYAN, BLACK);
        }
    } else {
        row_put(2, "T:------------", GRAY, BLACK);  /* 14 字符: "T:" + 12 横线 */
        row_put(3, "", CYAN, BLACK);
    }

    /* ROW 4/5: RX HEX
     *   HEAD(2) ADDR(2) FUNC(2) ERR(2) = 8 字符 + "R:" 前缀 = 10 字符 (主行)
     *   续行 4 字节 HEX = 8 字符 */
    if (f->data_len > 0 || f->function_code != 0) {
        snprintf(line, sizeof(line), "R:%02X%02X%02X%02X",
                 PD42S1_FRAME_HEAD, f->slave_addr, f->function_code, f->data[0]);
        row_put(4, line, GREEN, BLACK);

        if (f->data_len > 4) {
            snprintf(line, sizeof(line), "%02X%02X%02X%02X",
                     f->data[1], f->data[2], f->data[3], f->data[4]);
            row_put(5, line, GREEN, BLACK);
        } else {
            row_put(5, "", GREEN, BLACK);
        }
    } else {
        row_put(4, "R:--------", GRAY, BLACK);  /* 10 字符 */
        row_put(5, "", GREEN, BLACK);
    }
}

/* ============================================================================
 * MotorPage 键处理 (原 main.c 派发, 但语义按最新需求重写)
 *
 *   K1 down (与 !K2)  → SM_Run(SM_X, R)
 *   K2 down (与 !K1)  → SM_Run(SM_X, L)
 *   K1/K2 up          → SM_Stop + SM_ReadPosition (停下后读一次位置)
 *   K3 down           → SM_zeroset    (保存当前位置为回零位置)
 *   K4 down           → SM_zero HM    (多圈回零触发)
 * ============================================================================ */
static void MotorPage_OnKey(void) {
    if (key(1, down) && !key_pressed(2)) {
        SM_Run(SM_X, R, 100, 60);   /* RUN_ACCEL=100, RUN_RPM=60 */
        UI_ForceRedraw();
    } else if (key(2, down) && !key_pressed(1)) {
        SM_Run(SM_X, L, 100, 60);
        UI_ForceRedraw();
    } else if (key(1, up) || key(2, up)) {
        SM_Stop(SM_X);              /* 立即刹车 + CLEAR_STATUS */
        SM_ReadPosition(SM_X);      /* 停下后自动重发 0x2A 直到应答或 500ms */
        UI_ForceRedraw();
    } else if (key(3, down)) {
        SM_zeroset(SM_X, OL, 10000, true);   /* 读当前位置 → 锁存 → 设 0x90 + 0x95/0x97/0x04 */
        UI_ForceRedraw();
    } else if (key(4, down)) {
        SM_zero(SM_X, HM);          /* 多圈回零触发 */
        UI_ForceRedraw();
    } else if (key(5, down)) {
        UI_ForceRedraw();
    }
}

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
 * 页面注册表 (编译期常量, 顺序 = K5 切页顺序)
 *   PAGE_COUNT=3 表示注册 3 页: [0] Empty, [1] Motor, [2] HuiduTB
 *   HuiduPage + TB6612Page 合并成 HuiduTBPage (用户 02:55 反馈: 和 TB6612 放同一页)
 * ============================================================================ */
const page_t pages[] = {
    { "----",   EmptyPage_Render, EmptyPage_OnKey, NULL },                          /* [0] 空页 (占位) */
    { "MOTOR",  MotorPage_Render, MotorPage_OnKey, NULL },                          /* [1] PD42S1 闭环步进电机 */
    { "HUITB",  HuiduTBPage_Render, HuiduTBPage_OnKey, NULL },                     /* [2] 灰度 + TB6612 编码电机 */
};

void UI_Init(void) {
    g_page = 0;
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

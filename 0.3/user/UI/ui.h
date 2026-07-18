/* ============================================================================
 * @file    ui.h
 * @brief   多页面 LCD 渲染 (128x160 竖屏, 8x16 字体)
 *
 *   框架:
 *     - page_t { name, render, on_key } 逐页注册到 pages[]
 *     - UI_Render() 每帧: 整屏清黑 → 当前页 render() → 在最后一行画页脚
 *     - K5 仅做切页 (循环: 末页按下 → 回到第 0 页), 切页副作用: SM_Stop(SM_X)
 *     - K1~K4 由当前页 on_key() 处理 (空页时 on_key 是空函数, 自动忽略)
 *
 *   LCD 行布局 (行高 16, 共 10 行):
 *     y=  0  .. y=144  各页自定义 (最多 9 行内容)
 *     y=144           页脚: "P<cur+1>/<PAGE_COUNT> <name>", 白字深蓝底 (第 10 行)
 *
 * ============================================================================
 * 调用方法 (main.c 主循环用法)
 * ============================================================================
 *
 *   上电调一次:
 *     UI_Init();                         // g_page=0, 首帧 dirty
 *
 *   主循环每帧:
 *     UI_Render();                       // 当前页 render + 页脚 + hash 缓存
 *
 *   切页 / 按键触发瞬时刷新:
 *     UI_ForceRedraw();                  // mark_all_rows_dirty, 下一帧整页重画
 *
 *   全局状态:
 *     g_page                             当前页 0..PAGE_COUNT-1 (main.c K5 推进)
 *     PAGE_COUNT                         pages[] 注册表长度 (编译期常量)
 *     pages[]                            注册表 (在 ui.c 中定义)
 *
 * ============================================================================
 */
#ifndef __UI_H__
#define __UI_H__

#include <stdint.h>

/* 行 y 起点宏 (供各页 render 复用)
 *   行高 16 px (与字模 LCD_8X16 一致, 无额外间距), 共 10 行
 *   y = n * 16, n = 0..9, 第 10 行 y = 144 高 16 → y = 160 恰好铺满 LCD_H */
#define ROW_Y(n)  ((n) * 16)

/* ============================================================================
 * 页面框架
 * ============================================================================ */

/* 单页描述: 名字 + render + 键处理 + 可选自定义页脚.
 *   name        — 缺省页脚文字 ("P<n>/<N> <name>"), 整页脚格局保持一致
 *   render      — 渲染 ROW 0..8 内容
 *   on_key      — K1~K4 处理 (空页可空函数)
 *   footer_hook — NULL: 用缺省页脚; 非 NULL: 调用此函数, 函数内部 row_put(9, ...) 自定义页脚
 *                 (用于挤掉页脚塞数据, 如 TB6612 页 "当前 PWM")
 */
typedef struct {
    const char *name;              /* 缺省页脚名, ≤ 8 字符 */
    void      (*render)(void);
    void      (*on_key)(void);
    void      (*footer_hook)(void); /* NULL → 用 "P<n>/<N> <name>"; 否则用此函数全权画 ROW 9 */
} page_t;

/* 当前页下标 (0..PAGE_COUNT-1), main.c 在 K5 down 时推进 + 循环回 0 */
extern uint8_t g_page;

/* 当前注册的页面总数 (编译期 = 注册表长度) */
extern const uint8_t PAGE_COUNT;

/* 注册表 (main.c 通过本页头引用, 各页在 ui.c 中定义) */
extern const page_t pages[];

/* UI 初始化: 当前页归 0 (pages 表在编译期已注册, 不需运行时注册) */
void UI_Init(void);

/* 主渲染: 每帧调用一次, 内部完成清屏 + 当前页 render + 页脚 (默认 60ms 节流) */
void UI_Render(void);

/* 唤醒一帧强制重绘 (60ms 节流对按键/读位置等高优先级事件失效) */
void UI_ForceRedraw(void);

#endif

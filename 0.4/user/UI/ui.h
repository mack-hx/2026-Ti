/* ============================================================================
 * @file    ui.h
 * @brief   多页面 LCD 渲染 (128x160 竖屏, 8x16 字体) - 菜单 + 详情页架构
 *
 *   框架:
 *     - 两层状态: g_mode (menu / detail)
 *     - detail 模式: g_page 指向 pages[] 当前详情页 (0..PAGE_COUNT-1, 0 是菜单)
 *     - menu   模式: g_menu_sel 指向菜单选中项 (1..MENU_ITEM_COUNT)
 *     - K5 行为: 详情页 → 进菜单; 菜单页 → 进入选中项 (g_page = g_menu_sel)
 *     - K1/K2 仅在菜单页有效: 上下切换 g_menu_sel (循环)
 *
 *   页面注册表 pages[] (顺序 = 菜单显示顺序):
 *     [0]  MenuPage     菜单页 (详情页"home")
 *     [1]  Task1Page    题目页 1 (placeholder)
 *     [2]  Task2Page    题目页 2 (placeholder)
 *     [3]  Task3Page    题目页 3 (placeholder)
 *     [4]  Task4Page    题目页 4 (placeholder, 预留)
 *     [5]  Task5Page    题目页 5 (placeholder, 预留)
 *     [6]  MotorXPage   PD42S1 X 轴硬件调试
 *     [7]  MotorYPage   PD42S1 Y 轴硬件调试
 *     [8]  HuiduTBPage  灰度 + TB6612 编码电机 (硬件页 3)
 *     [9]  MPU9250Page  9 轴 IMU (硬件页 4)
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
 *     UI_Init();                         // g_page=0 (菜单页), 首帧 dirty
 *
 *   主循环每帧:
 *     UI_Render();                       // 当前页 render + 页脚 + hash 缓存
 *
 *   切页 / 按键触发瞬时刷新:
 *     UI_ForceRedraw();                  // mark_all_rows_dirty, 下一帧整页重画
 *
 *   全局状态:
 *     g_page                             当前详情页 0..PAGE_COUNT-1 (0 = 菜单)
 *     PAGE_COUNT                         pages[] 注册表长度 (编译期常量)
 *     pages[]                            注册表 (在 ui.c 中定义)
 *     g_menu_sel                         菜单页选中项 (1..MENU_ITEM_COUNT)
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
 * 菜单项配置
 * ============================================================================
 *   MENU_ITEM_COUNT = 菜单项总数 (不含菜单页本身)
 *   菜单项下标 1..MENU_ITEM_COUNT 对应 pages[] 的 [1..MENU_ITEM_COUNT]
 *   菜单项显示顺序:
 *     1: Task1   题目页 1
 *     2: Task2   题目页 2
 *     3: Task3   题目页 3
 *     4: Task4   题目页 4 (预留)
 *     5: Task5   题目页 5 (预留)
 *     6: MOTOR-X 硬件页: X 轴步进电机
 *     7: MOTOR-Y 硬件页: Y 轴步进电机
 *     8: HUITB   硬件页: 灰度 + TB6612
 *     9: MPU9250 硬件页: 9 轴 IMU
 * ============================================================================ */
#define MENU_ITEM_COUNT   9U   /* 5 题目 + 4 硬件 */

/* ============================================================================
 * 页面框架
 * ============================================================================
 * 单页描述: 名字 + render + 键处理 + 可选自定义页脚.
 *   name        — 缺省页脚文字 ("P<n>/<N> <name>"), 整页脚格局保持一致
 *   render      — 渲染 ROW 0..8 内容
 *   on_key      — K1~K4 处理 (空页可空函数)
 *   footer_hook — NULL: 用缺省页脚; 非 NULL: 调用此函数, 函数内部 row_put(9, ...) 自定义页脚
 *                 (用于挤掉页脚塞数据, 如 TB6612 页 "当前 PWM")
 *   注: 菜单页 (g_page=0) 的 K1~K4 行为由 MenuPage_OnKey 接管, 与 detail 页的 on_key 不冲突
 */
typedef struct {
    const char *name;              /* 缺省页脚名, ≤ 8 字符 */
    void      (*render)(void);
    void      (*on_key)(void);
    void      (*footer_hook)(void); /* NULL → 用 "P<n>/<N> <name>"; 否则用此函数全权画 ROW 9 */
} page_t;

/* 当前详情页下标 (0..PAGE_COUNT-1, 0 = 菜单页), main.c K5 在 detail 模式下回 0 */
extern uint8_t g_page;

/* 菜单页选中项 (1..MENU_ITEM_COUNT, 上电默认 1 即 Task1), main.c K1/K2 在菜单页推进 */
extern uint8_t g_menu_sel;

/* 当前注册的详情页总数 (编译期 = 注册表长度, 含菜单页 #0) */
extern const uint8_t PAGE_COUNT;

/* 注册表 (main.c 通过本页头引用, 各页在 ui.c 中定义) */
extern const page_t pages[];

/* UI 初始化: g_page=0 (菜单页), g_menu_sel=1 (默认选中 Task1), 首帧 dirty */
void UI_Init(void);

/* 主渲染: 每帧调用一次, 内部完成清屏 + 当前页 render + 页脚 (默认 60ms 节流) */
void UI_Render(void);

/* 唤醒一帧强制重绘 (60ms 节流对按键/读位置等高优先级事件失效) */
void UI_ForceRedraw(void);

#endif
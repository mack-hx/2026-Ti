/* ============================================================================
 * @file    ui.h
 * @brief   LCD 多页面 UI 框架 (128x160 竖屏, 8x16 字体, 行高 16 px)
 *
 *   - 渲染框架: 按行缓存 + 16-bit FNV-1a 哈希, 内容不变的行零开销
 *   - 两层页面: g_page == 0 是菜单, 1..PAGE_COUNT-1 是详情页 (主页占 [1])
 *   - 注册表: pages[] 在 ui.c 中定义, render/on_key/footer_hook 各页实现见 ui.c
 *   - main.c 主循环只需 UI_Render(), 不直接接触任何具体页逻辑
 *
 *   LCD 几何:
 *     128 px 宽 × 160 px 高, 8 px 等宽字体 LCD_8X16
 *     一行最多 16 字符, 行高 16 px, 共 10 行铺满
 *     y = n * 16, n ∈ [0..9]
 *
 *   完整 API (见 ui.c):
 *     UI_Init()       - 上电调一次
 *     UI_Render()     - 主循环每帧调
 *     UI_ForceRedraw()- 切页 / 按键时强制全屏重画
 *
 * ============================================================================
 */
#ifndef __UI_H__
#define __UI_H__

#include <stdint.h>
#include <stdbool.h>

/* 行起点宏: ROW_Y(n) = n * 16 (LCD 行高 16 px) */
#define ROW_Y(n)  ((n) * 16)

/* 一行最多 16 字符 (128 px / 8 px 字体) */
#define LCD_ROW_CHARS  16U

/* 菜单项总数 (不含菜单页 + 主页; kMenuNames[] 长度 = MENU_ITEM_COUNT)
 *
 *   pages[] 顺序: [0]Menu / [1]main / [2..6]Task1..5 / [7]Motor-X / [8]Motor-Y
 *                 / [9]HuiduTB / [10]MPU9250 (PAGE_COUNT=11)
 *
 *   g_page=1 = 上电默认进主页 (main), 不在菜单里
 *   g_menu_sel ∈ [2, 1+MENU_ITEM_COUNT] (10) = 菜单选中 TASK1..MPU9250 之一
 *
 *   主页 (g_page=1) 不通过菜单进入, 上电直进; 想回主页在菜单页按 K5 选中
 *   TASK1..MPU9250 都不行, 必须从菜单退出上电重启 (暂时简化, 后需单独加 menu 选项)
 */
/* 菜单项总数 (= kMenuNames[] 长度)
 *
 *   pages[] 顺序:
 *     [0]  MenuPage                       (菜单页本身, 不计入 kMenuNames)
 *     [1]  MainPage  (主页 = 所有外设状态总览)
 *     [2..6] Task1..5Page
 *     [7]  MotorXPage
 *     [8]  MotorYPage
 *     [9]  HuiduTBPage
 *     [10] MPU9250Page
 *     PAGE_COUNT = 11
 *
 *   kMenuNames[10]: "MAIN / TASK1..TASK5 / MOTOR-X / MOTOR-Y / HUITB / MPU9250"
 *   g_menu_sel ∈ [1, MENU_ITEM_COUNT] (10), 上电默认 MAIN_BOOT_MENU_SEL=2 (TASK1)
 *
 *   菜单 "MAIN" (g_menu_sel=1) → K5 进入 pages[1] = 主页
 *   主页 (g_page=1) 是上电默认入口, K5 退到菜单页
 */
#define MENU_ITEM_COUNT  10U

/* 单页描述: 名字 + 渲染 + 键处理 + 可选自定义页脚 */
typedef struct {
    const char *name;                /* 缺省页脚名 */
    void      (*render)(void);
    void      (*on_key)(void);
    void      (*footer_hook)(void);  /* NULL → 缺省页脚 "P<n>/<N> <name>"; 否则此函数自管 ROW 9 */
} page_t;

/* 当前页: 0 = 菜单, 1..PAGE_COUNT-1 = 详情页 */
extern uint8_t g_page;

/* 菜单页选中项: 1..MENU_ITEM_COUNT, 上电默认 MAIN_BOOT_MENU_SEL (= 2 = TASK1) */
extern uint8_t g_menu_sel;

/* 注册表页数 (含菜单页 #0) */
extern const uint8_t PAGE_COUNT;

/* 注册表 (在 ui.c 中定义, render/on_key 实现见 app_pages.c) */
extern const page_t pages[];

void UI_Init(void);
void UI_Render(void);
void UI_ForceRedraw(void);

#endif /* __UI_H__ */

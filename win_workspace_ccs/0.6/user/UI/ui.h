/* ============================================================================
 * ui.h - LCD 多页面 UI 框架
 *
 * 128x160 竖屏, 8x16 字体, 行高 16px, 共 10 行
 *
 * 架构:
 *   g_page == 0     → 菜单页
 *   g_page == 1..10 → 详情页 (主页/Task/Motor/Huitb/MPU9250)
 *   pages[]         → 注册表 (render / on_key / footer_hook)
 *
 * 调用:
 *   UI_Init()       → 上电一次
 *   UI_Render()     → 每帧
 *   UI_ForceRedraw()→ 切页 / 按键
 * ============================================================================
 */
#ifndef __UI_H__
#define __UI_H__

#include <stdint.h>
#include <stdbool.h>

/* LCD 行高 16px, 一行最多 16 字符 */
#define ROW_Y(n)        ((n) * 16)
#define LCD_ROW_CHARS   16U
#define MENU_ITEM_COUNT 10U   // kMenuNames 长度

/* 页面描述符 */
typedef struct {
    const char *name;                // 页脚名
    void      (*render)(void);       // 渲染函数
    void      (*on_key)(void);       // 按键处理 (NULL=noop)
    void      (*footer_hook)(void);  // 自定义页脚 (NULL=缺省)
} page_t;

/* 全局状态 */
extern uint8_t       g_page;      // 当前页
extern uint8_t       g_menu_sel; // 菜单选中项
extern const uint8_t PAGE_COUNT;  // 总页数
extern const page_t  pages[];     // 页面注册表

/* 公开 API */
void UI_Init(void);
void UI_Render(void);
void UI_ForceRedraw(void);

#endif /* __UI_H__ */

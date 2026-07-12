/**
 * @file    ui.h
 * @brief   LCD 显示渲染 (基于 sm_* 全局变量 + key_pressed() 5 键状态)
 *
 *   LCD 布局 (128x160 竖屏, 6 行紧凑):
 *     y=  0  键位: k:1 1 1 1 1   (K1 K2 K3 K4 K5)
 *     y= 18  POS  ERR  STATE
 *     y= 36  TX   [b0..b3]
 *     y= 54  TX   [b4..b7]  (短帧时清屏)
 *     y= 72  RX   [HEAD ADDR FUNC ERR]
 *     y= 90  RX   [data[1..4]] (读位置应答位置四字节)
 */
#ifndef __UI_H__
#define __UI_H__

#include <stdint.h>

/* 主渲染: 读 sm_pos/sm_err/sm_state + 5 个按键, 画到 LCD (每帧调用) */
void UI_Render(void);

#endif

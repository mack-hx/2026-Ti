/*TI移植指南***********************
GPIO方面:
    1.添加一个GPIO,将Name改为 LCD,将Port改为PORTB 
    2.添加四个Group Pins,分别命名为 RES 、 DC 、 CS 、 BLK
    3.将RES和DC的初始电平(Initial Value)设为Set(高),CS和BLK设置为Cleared(低)
    4.将引脚改为RES(PB10)、DC(PB11)、CS(PB14)、BLK(PB26)
SPI方面:
    1.添加一个SPI,将Name改为 SPI_LCD
    2.将Target Bit Rate (Hz) 改为 20000000(需要将主频改为80MHZ)
    3.将Frame Format改为 Motorola 3-wire
    4.将SPI Peripheral 改为 SPI1
    5.将SPI SCLK (Clock) 设为 PB9, SPI PICO 设为PB8, SPI POCI可自定义，屏幕用不到
*******************************/
#ifndef __LCD_H
#define __LCD_H

#include "system/clock.h"//延时头文件
#include <string.h>
#include "LCD_Data.h"
#include "ti_msp_dl_config.h"

/*参数宏定义*********************/

#define X_Offset 0  //X轴偏移量
#define Y_Offset 0  //Y轴偏移量

/*sizey参数取值*/
/*此参数值不仅用于判断，而且用于计算横向字符偏移，默认值为字体像素宽度*/
#define LCD_6X12				12
#define LCD_8X16				16
#define LCD_12X24				24
#define LCD_16X32				32

/*sizey参数取值*/
#define LCD_modeoff				0
#define LCD_modeon				1

/*IsFilled参数数值*/
#define LCD_UNFILLED			0
#define LCD_FILLED				1

#define USE_HORIZONTAL 1  //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏
#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 128
#define LCD_H 160

#else
#define LCD_W 160
#define LCD_H 128
#endif

//画笔颜色
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE           	 0x001F  
#define BRED             0XF81F
#define GRED 			       0XFFE0
#define GBLUE			       0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			     0XBC40 //棕色
#define BRRED 			     0XFC07 //棕红色
#define GRAY  			     0X8430 //灰色
#define DARKBLUE      	 0X01CF	//深蓝色
#define LIGHTBLUE      	 0X7D7C	//浅蓝色  
#define GRAYBLUE       	 0X5458 //灰蓝色
#define LIGHTGREEN     	 0X841F //浅绿色
#define LGRAY 			     0XC618 //浅灰色(PANNEL),窗体背景色
#define LGRAYBLUE        0XA651 //浅灰蓝色(中间层颜色)
#define LBBLUE           0X2B12 //浅棕蓝色(选择条目的反色)

/*********************参数宏定义*/


/*函数声明*********************/

/*初始化函数*/
void LCD_Init(uint16_t color);//LCD初始化函数

/*显示函数*/
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);//LCD显示单个字符函数
void LCD_ShowString(uint16_t X, uint16_t Y, char *String, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);//LCD显示字符串（支持ASCII码和中文混合写入）
void LCD_ShowIntNum(uint16_t x,uint16_t y,uint16_t num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey);//LCD显示整数变量
void LCD_ShowFloatNum(uint16_t x,uint16_t y,float num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey);//LCD显示两位小数变量
void LCD_ShowPicture(uint16_t x,uint16_t y,uint16_t length,uint16_t width,const uint8_t pic[]);//LCD显示图片函数
void LCD_Printf(int16_t X, int16_t Y, uint16_t fc, uint16_t bc, uint8_t FontSize, uint8_t mode, char *format, ...);

/*绘图函数*/
void LCD_DrawPoint(uint16_t x,uint16_t y,uint16_t color);//LCD在指定位置画点函数
void LCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color);//LCD在指定区域填充颜色函数

#endif

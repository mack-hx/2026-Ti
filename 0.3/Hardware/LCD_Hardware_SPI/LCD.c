/* ============================================================================
 * @file    LCD.c
 * @brief   1.8寸 TFT LCD 移植驱动 (SPI1 Motorola 3-wire, 20MHz, 竖屏 128×160)
 *          基于 ST7735S 控制器 (移植自江协科技/正点原子模板)
 *
 * ============================================================================
 * 调用方法 (主循环用法) - 应用层应直接用 ui.c 的 row_put() / row_flush()
 *                     间接调用本文件, 避免自己拼字符串再调 ShowString.
 *                     本文件提供底层 API 给 ui.c 用.
 * ============================================================================
 *
 *   上电 (main() 启动序列里调一次):
 *     LCD_Init(BLUE);                       // 清屏, 整屏填 BLUE, 开背光
 *
 *   画图 (通常 ui.c 用这些):
 *     LCD_Fill(x0, y0, x1, y1, color);      // 矩形填充 (含端点), 用于清一行
 *     LCD_DrawPoint(x, y, color);           // 单点
 *
 *   字符串 (字体 8x16 / 6x12 / 12x24 / 16x32, mode=0/1):
 *     LCD_ShowChar(x, y, 'A', fc, bc, LCD_8X16, 0);
 *     LCD_ShowString(x, y, "hello", fc, bc, LCD_8X16, 0);
 *     LCD_ShowIntNum(x, y, 123, 4, fc, bc, LCD_8X16);   // 4 位整数
 *     LCD_ShowFloatNum(x, y, 1.23, 3, fc, bc, LCD_8X16); // 浮点
 *     LCD_Printf(x, y, fc, bc, LCD_8X16, 0, "P:%d", pos);
 *
 *   图片:
 *     LCD_ShowPicture(x, y, length, width, pic);  // length=宽, width=高
 *
 *   颜色常量 (16-bit RGB565): WHITE / BLACK / BLUE / RED / GREEN / CYAN /
 *     YELLOW / MAGENTA / GRAY / LGRAY / DARKBLUE ... 详见 LCD.h
 *
 *   字体大小:  LCD_6X12 / LCD_8X16 / LCD_12X24 / LCD_16X32
 *   mode:     LCD_modeoff (0) / LCD_modeon (1)
 *   屏幕方向: USE_HORIZONTAL=0 (竖屏 128×160) 或 1 (竖屏翻转) 或 2/3 (横屏)
 *
 * ============================================================================
 *
 * LCD 布局约束 (128×160 竖屏, 字体 8x16):
 *   每行最多 16 字符 (128 / 8 = 16)
 *   共 10 行        (160 / 16 = 10)
 *   行高 16 px, 行间距 2 px → 每行占 18 px
 *   写 UI 时所有 snprintf / LCD_ShowString 必须按 16 字符上限规划, 不许超界
 *
 * ============================================================================
 */
#include "LCD.h"
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>

static void LCD_Delay_ms(unsigned long num_ms)
{
	mspm0_delay_ms(num_ms);//改成自己的延时函数
}

/*引脚配置*********************/

/**
  * 函    数：LCD操纵复位函数
  * 参    数：无
  * 返 回 值：无
  * 说    明：无
  */
void LCD_W_RES(uint8_t BitValue)
{
	/*根据BitValue的值，将RES置高电平或者低电平*/
	if (BitValue == 1) 
	{
		DL_GPIO_setPins(LCD_PORT, LCD_RES_PIN);    // 输出高电平
	} 
	else 
	{
		DL_GPIO_clearPins(LCD_PORT, LCD_RES_PIN);  // 输出低电平
	}
}

/**
  * 函    数：LCD操纵数据/命令选择线函数
  * 参    数：无
  * 返 回 值：无
  * 说    明：无
  */
void LCD_W_DC(uint8_t BitValue)
{
	/*根据BitValue的值，将DC置高电平或者低电平*/
	if (BitValue == 1) 
	{
		DL_GPIO_setPins(LCD_PORT, LCD_DC_PIN);    // 输出高电平
	} 
	else 
	{
		DL_GPIO_clearPins(LCD_PORT, LCD_DC_PIN);  // 输出低电平
	}
}

/**
  * 函    数：LCD操纵片选函数
  * 参    数：无
  * 返 回 值：无
  * 说    明：无
  */
void LCD_W_CS(uint8_t BitValue)
{
	/*根据BitValue的值，将CS置高电平或者低电平*/
	if (BitValue == 1) 
	{
		DL_GPIO_setPins(LCD_PORT, LCD_CS_PIN);    // 输出高电平
	} 
	else 
	{
		DL_GPIO_clearPins(LCD_PORT, LCD_CS_PIN);  // 输出低电平
	}
}

/**
  * 函    数：LCD操纵背光函数
  * 参    数：无
  * 返 回 值：无
  * 说    明：无
  */
void LCD_W_BLK(uint8_t BitValue)
{
	/*根据BitValue的值，将CS置高电平或者低电平*/
	if (BitValue == 1) 
	{
		DL_GPIO_setPins(LCD_PORT, LCD_BLK_PIN);    // 输出高电平
	} 
	else 
	{
		DL_GPIO_clearPins(LCD_PORT, LCD_BLK_PIN);  // 输出低电平
	}
}

/**
  * 函    数：LCD写入8位数据函数
  * 参    数dat：要写入的8位数据
  * 返 回 值：无
  * 说    明：无
  */
void LCD_Send8Bit(uint8_t dat)
{
	LCD_W_CS(0);
    //发送数据
    DL_SPI_transmitData8(SPI_LCD_INST, dat);
    //等待SPI总线空闲
    while(DL_SPI_isBusy(SPI_LCD_INST));
    LCD_W_CS(1);	
}

/**
  * 函    数：LCD写入16位数据函数
  * 参    数dat：要写入的16位数据
  * 返 回 值：无
  * 说    明：无
  */
static void LCD_Send16Bit(uint16_t dat)
{
	LCD_Send8Bit(dat>>8);  // 先发送高8位
	LCD_Send8Bit(dat);     // 再发送低8位
}

/**
  * 函    数：LCD写入命令函数
  * 参    数dat：写入的命令
  * 返 回 值：无
  * 说    明：无
  */
static void LCD_WriteCommand(uint8_t dat)
{
	LCD_W_DC(0);    // DC线拉低，表示接下来发送的是命令
	LCD_Send8Bit(dat);  // 发送命令字节
	LCD_W_DC(1);    // DC线拉高，表示后续发送的是数据
}

/**
  * 函    数：LCD设置起始和结束地址
  * 参    数x1,x2：设置列的起始和结束地址
  * 参    数y1,y2：设置行的起始和结束地址
  * 返 回 值：无
  * 说    明：无
  */
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{   
	if(USE_HORIZONTAL == 0 || USE_HORIZONTAL == 1)
	{
		LCD_WriteCommand(0x2a);//列地址设置
		LCD_Send16Bit(x1 + Y_Offset);
		LCD_Send16Bit(x2 + Y_Offset);
		LCD_WriteCommand(0x2b);//行地址设置
		LCD_Send16Bit(y1 + X_Offset);
		LCD_Send16Bit(y2 + X_Offset);
		LCD_WriteCommand(0x2c);//储存器写
	}
	else
	{
		LCD_WriteCommand(0x2a);//列地址设置
		LCD_Send16Bit(x1 + X_Offset);
		LCD_Send16Bit(x2 + X_Offset);
		LCD_WriteCommand(0x2b);//行地址设置
		LCD_Send16Bit(y1 + Y_Offset);
		LCD_Send16Bit(y2 + Y_Offset);
		LCD_WriteCommand(0x2c);//储存器写
	}
}

/**
  * 函    数：LCD初始化函数
  * 参    数color：给背景填充的颜色
  * 返 回 值：无
  * 说    明：无
  */
void LCD_Init(uint16_t color)
{
	LCD_W_RES(0);//复位
	LCD_Delay_ms(100);
	LCD_W_RES(1);
	LCD_Delay_ms(100);
	
	//************* Start Initial Sequence **********//
	LCD_WriteCommand(0x11); //Sleep out 
	LCD_Delay_ms(120);              //Delay 120ms 
	//------------------------------------ST7735S Frame Rate-----------------------------------------// 
	LCD_WriteCommand(0xB1); 
	LCD_Send8Bit(0x05); 
	LCD_Send8Bit(0x3C); 
	LCD_Send8Bit(0x3C); 
	LCD_WriteCommand(0xB2); 
	LCD_Send8Bit(0x05);
	LCD_Send8Bit(0x3C); 
	LCD_Send8Bit(0x3C); 
	LCD_WriteCommand(0xB3); 
	LCD_Send8Bit(0x05); 
	LCD_Send8Bit(0x3C); 
	LCD_Send8Bit(0x3C); 
	LCD_Send8Bit(0x05); 
	LCD_Send8Bit(0x3C); 
	LCD_Send8Bit(0x3C); 
	//------------------------------------End ST7735S Frame Rate---------------------------------// 
	LCD_WriteCommand(0xB4); //Dot inversion 
	LCD_Send8Bit(0x03); 
	//------------------------------------ST7735S Power Sequence---------------------------------// 
	LCD_WriteCommand(0xC0); 
	LCD_Send8Bit(0x28); 
	LCD_Send8Bit(0x08); 
	LCD_Send8Bit(0x04); 
	LCD_WriteCommand(0xC1); 
	LCD_Send8Bit(0XC0); 
	LCD_WriteCommand(0xC2); 
	LCD_Send8Bit(0x0D); 
	LCD_Send8Bit(0x00); 
	LCD_WriteCommand(0xC3); 
	LCD_Send8Bit(0x8D); 
	LCD_Send8Bit(0x2A); 
	LCD_WriteCommand(0xC4); 
	LCD_Send8Bit(0x8D); 
	LCD_Send8Bit(0xEE); 
	//---------------------------------End ST7735S Power Sequence-------------------------------------// 
	LCD_WriteCommand(0xC5); //VCOM 
	LCD_Send8Bit(0x1A); 
	LCD_WriteCommand(0x36); //MX, MY, RGB mode 
	if(USE_HORIZONTAL==0)LCD_Send8Bit(0x00);
	else if(USE_HORIZONTAL==1)LCD_Send8Bit(0xC0);
	else if(USE_HORIZONTAL==2)LCD_Send8Bit(0x70);
	else LCD_Send8Bit(0xA0); 
	//------------------------------------ST7735S Gamma Sequence---------------------------------// 
	LCD_WriteCommand(0xE0); 
	LCD_Send8Bit(0x04); 
	LCD_Send8Bit(0x22); 
	LCD_Send8Bit(0x07); 
	LCD_Send8Bit(0x0A); 
	LCD_Send8Bit(0x2E); 
	LCD_Send8Bit(0x30); 
	LCD_Send8Bit(0x25); 
	LCD_Send8Bit(0x2A); 
	LCD_Send8Bit(0x28); 
	LCD_Send8Bit(0x26); 
	LCD_Send8Bit(0x2E); 
	LCD_Send8Bit(0x3A); 
	LCD_Send8Bit(0x00); 
	LCD_Send8Bit(0x01); 
	LCD_Send8Bit(0x03); 
	LCD_Send8Bit(0x13); 
	LCD_WriteCommand(0xE1); 
	LCD_Send8Bit(0x04); 
	LCD_Send8Bit(0x16); 
	LCD_Send8Bit(0x06); 
	LCD_Send8Bit(0x0D); 
	LCD_Send8Bit(0x2D); 
	LCD_Send8Bit(0x26); 
	LCD_Send8Bit(0x23); 
	LCD_Send8Bit(0x27); 
	LCD_Send8Bit(0x27); 
	LCD_Send8Bit(0x25); 
	LCD_Send8Bit(0x2D); 
	LCD_Send8Bit(0x3B); 
	LCD_Send8Bit(0x00); 
	LCD_Send8Bit(0x01); 
	LCD_Send8Bit(0x04); 
	LCD_Send8Bit(0x13); 
	//------------------------------------End ST7735S Gamma Sequence-----------------------------// 
	LCD_WriteCommand(0x3A); //65k mode 
	LCD_Send8Bit(0x05); 
	LCD_WriteCommand(0x29); //Display on 
	LCD_Fill(0,0,LCD_W,LCD_H,color);//填充背景为白色
	
	LCD_W_BLK(1);//打开背光
	LCD_Delay_ms(100);
} 

/**
  * 函    数：LCD在指定位置画点函数
  * 参    数x,y：画点坐标
  * 参    数color：点的颜色
  * 返 回 值：无
  * 说    明：无
  */
void LCD_DrawPoint(uint16_t x,uint16_t y,uint16_t color)
{
	LCD_Address_Set(x,y,x,y);//设置光标位置 
	LCD_Send16Bit(color);
} 

/**
  * 函    数：LCD在指定区域填充颜色函数
  * 参    数xsta,ysta：起始坐标
  * 参    数xend,yend：终止坐标
  * 参    数color： 要填充的颜色
  * 返 回 值：无
  * 说    明：无
  */
void LCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color)
{          
	uint16_t i,j; 
	LCD_Address_Set(xsta, ysta, xend, yend);//设置显示范围
	for(i=ysta;i<yend;i++)
	{													   	 	
		for(j=xsta;j<xend;j++)
		{
			LCD_Send16Bit(color);
		}
	} 					  	    
}

/**
  * 函    数:LCD显示单个字符函数
  * 参    数 x,y:显示坐标
  * 参    数 num:终止坐标
  * 参    数 fc:字的颜色
  * 参    数 bc:字的背景色
  * 参    数 sizey:字号
  * 参    数 mode:0非叠加模式  1叠加模式
  * 返 回 值：无
  * 说    明：无
  */
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t temp,sizex,t,m=0;
	uint16_t i,TypefaceNum;//一个字符所占字节大小
	uint16_t x0=x;
	sizex=sizey/2;
	TypefaceNum=(sizex/8+((sizex%8)?1:0))*sizey;
	num=num-' ';    //得到偏移后的值
	LCD_Address_Set(x,y,x+sizex-1,y+sizey-1);  //设置光标位置 
	for(i=0;i<TypefaceNum;i++)
	{ 
		if(sizey==12)temp=LCD_F6X12[num][i];		       //调用6x12字体
		else if(sizey==16)temp=LCD_F8X16[num][i];		 //调用8x16字体
		else if(sizey==24)temp=LCD_F12X24[num][i];		 //调用12x24字体
		else if(sizey==32)temp=LCD_F16X32[num][i];		 //调用16x32字体
		else return;
		for(t=0;t<8;t++)
		{
			if(!mode)//非叠加模式
			{
				if(temp&(0x01<<t))LCD_Send16Bit(fc);
				else LCD_Send16Bit(bc);
				m++;
				if(m%sizex==0)
				{
					m=0;
					break;
				}
			}
			else//叠加模式
			{
				if(temp&(0x01<<t))LCD_DrawPoint(x,y,fc);//画一个点
				x++;
				if((x-x0)==sizex)
				{
					x=x0;
					y++;
					break;
				}
			}
		}
	}   	 	  
}

/**
  * 函    数:LCD显示图片函数
  * 参    数 x,y:起点坐标
  * 参    数 length:图片长度
  * 参    数 width:图片宽度
  * 参    数 pic[]:图片数组
  * 返 回 值：无
  * 说    明：该函数无需外部调用
  */
void LCD_ShowPicture(uint16_t x,uint16_t y,uint16_t length,uint16_t width,const uint8_t pic[])
{
	uint16_t i,j;
	uint32_t k=0;
	LCD_Address_Set(x,y,x+length-1,y+width-1);
	for(i=0;i<length;i++)
	{
		for(j=0;j<width;j++)
		{
			LCD_Send8Bit(pic[k*2]);
			LCD_Send8Bit(pic[k*2+1]);
			k++;
		}
	}			
}

/**
  * 函    数:显示单个12x12汉字
  * 参    数 x,y:显示坐标
  * 参    数 s:要显示的汉字坐标
  * 参    数 fc:字的颜色
  * 参    数 bc:字的背景色
  * 参    数 sizey:字号
  * 参    数 mode:0非叠加模式  1叠加模式
  * 返 回 值：无
  * 说    明：该函数无需外部调用
  */
void LCD_ShowChinese12x12(uint16_t x,uint16_t y,uint16_t s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)	
{
	uint8_t i,j,m=0;
	uint16_t TypefaceNum;//一个字符所占字节大小
	uint16_t x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;           
	LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
	for(i=0;i<TypefaceNum;i++)
	{
		for(j=0;j<8;j++)
		{	
			if(!mode)//非叠加方式
			{
				if(LCD_tfont12[s].Data[i]&(0x01<<j))LCD_Send16Bit(fc);
				else LCD_Send16Bit(bc);
				m++;
				if(m%sizey==0)
				{
					m=0;
					break;
				}
			}
			else//叠加方式
			{
				if(LCD_tfont12[s].Data[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
				x++;
				if((x-x0)==sizey)
				{
					x=x0;
					y++;
					break;
				}
			}
		}
	}			  	
} 

/**
  * 函    数:显示单个16x16汉字
  * 参    数 x,y:显示坐标
  * 参    数 s:要显示的汉字坐标
  * 参    数 fc:字的颜色
  * 参    数 bc:字的背景色
  * 参    数 sizey:字号
  * 参    数 mode:0非叠加模式  1叠加模式
  * 返 回 值：无
  * 说    明：该函数无需外部调用
  */
void LCD_ShowChinese16x16(uint16_t x,uint16_t y,uint16_t s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)	
{
	uint8_t i,j,m=0;
	uint16_t TypefaceNum;//一个字符所占字节大小
	uint16_t x0=x;
    TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;	
	LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
	for(i=0;i<TypefaceNum;i++)
	{
		for(j=0;j<8;j++)
		{	
			if(!mode)//非叠加方式
			{
				if(LCD_tfont16[s].Data[i]&(0x01<<j))LCD_Send16Bit(fc);
				else LCD_Send16Bit(bc);
				m++;
				if(m%sizey==0)
				{
					m=0;
					break;
				}
			}
			else//叠加方式
			{
				if(LCD_tfont16[s].Data[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
				x++;
				if((x-x0)==sizey)
				{
					x=x0;
					y++;
					break;
				}
			}
		}
	}
} 

/**
  * 函    数:显示单个24x24汉字
  * 参    数 x,y:显示坐标
  * 参    数 s:要显示的汉字坐标
  * 参    数 fc:字的颜色
  * 参    数 bc:字的背景色
  * 参    数 sizey:字号
  * 参    数 mode:0非叠加模式  1叠加模式
  * 返 回 值：无
  * 说    明：该函数无需外部调用
  */
void LCD_ShowChinese24x24(uint16_t x,uint16_t y,uint16_t s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)	
{
	uint8_t i,j,m=0;
	uint16_t TypefaceNum;//一个字符所占字节大小
	uint16_t x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
	for(i=0;i<TypefaceNum;i++)
	{
		for(j=0;j<8;j++)
		{	
			if(!mode)//非叠加方式
			{
				if(LCD_tfont24[s].Msk[i]&(0x01<<j))LCD_Send16Bit(fc);
				else LCD_Send16Bit(bc);
				m++;
				if(m%sizey==0)
				{
					m=0;
					break;
				}
			}
			else//叠加方式
			{
				if(LCD_tfont24[s].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
				x++;
				if((x-x0)==sizey)
				{
					x=x0;
					y++;
					break;
				}
			}
		}
	}
} 

/**
  * 函    数:显示单个32x32汉字
  * 参    数 x,y:显示坐标
  * 参    数 s:要显示的汉字坐标
  * 参    数 fc:字的颜色
  * 参    数 bc:字的背景色
  * 参    数 sizey:字号
  * 参    数 mode:0非叠加模式  1叠加模式
  * 返 回 值：无
  * 说    明：该函数无需外部调用
  */
void LCD_ShowChinese32x32(uint16_t x,uint16_t y,uint16_t s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)	
{
	uint8_t i,j,m=0;
	uint16_t TypefaceNum;//一个字符所占字节大小
	uint16_t x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
	for(i=0;i<TypefaceNum;i++)
	{
		for(j=0;j<8;j++)
		{	
			if(!mode)//非叠加方式
			{
				if(LCD_tfont32[s].Msk[i]&(0x01<<j))LCD_Send16Bit(fc);
				else LCD_Send16Bit(bc);
				m++;
				if(m%sizey==0)
				{
					m=0;
					break;
				}
			}
			else//叠加方式
			{
				if(LCD_tfont32[s].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
				x++;
				if((x-x0)==sizey)
				{
					x=x0;
					y++;
					break;
				}
			}
		}
	}
}

/**
  * 函    数:LCD显示字符串（支持ASCII码和中文混合写入）
  * 参    数 x,y:显示坐标
  * 参    数 *String:指定要显示的字符串
  * 参    数 fc:字的颜色
  * 参    数 bc:字的背景色
  * 参    数 sizey:指定字体大小
  * 参    数 mode:0非叠加模式  1叠加模式
  * 返 回 值：无
  * 说    明：无
  */
void LCD_ShowString(uint16_t X, uint16_t Y, char *String, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)	
{
	uint16_t i = 0;
	char SingleChar[5];
	uint8_t CharLength = 0;
	uint16_t XOffset = 0;
	uint16_t pIndex;
	
	while (String[i] != '\0')	//遍历字符串
	{
		
#ifdef LCD_CHARSET_UTF8						//定义字符集为UTF8
		/*此段代码的目的是，提取UTF8字符串中的一个字符，转存到SingleChar子字符串中*/
		/*判断UTF8编码第一个字节的标志位*/
		if ((String[i] & 0x80) == 0x00)			//第一个字节为0xxxxxxx
		{
			CharLength = 1;						//字符为1字节
			SingleChar[0] = String[i ++];		//将第一个字节写入SingleChar第0个位置，随后i指向下一个字节
			SingleChar[1] = '\0';				//为SingleChar添加字符串结束标志位
		}
		else if ((String[i] & 0xE0) == 0xC0)	//第一个字节为110xxxxx
		{
			CharLength = 2;						//字符为2字节
			SingleChar[0] = String[i ++];		//将第一个字节写入SingleChar第0个位置，随后i指向下一个字节
			if (String[i] == '\0') {break;}		//意外情况，跳出循环，结束显示
			SingleChar[1] = String[i ++];		//将第二个字节写入SingleChar第1个位置，随后i指向下一个字节
			SingleChar[2] = '\0';				//为SingleChar添加字符串结束标志位
		}
		else if ((String[i] & 0xF0) == 0xE0)	//第一个字节为1110xxxx
		{
			CharLength = 3;						//字符为3字节
			SingleChar[0] = String[i ++];
			if (String[i] == '\0') {break;}
			SingleChar[1] = String[i ++];
			if (String[i] == '\0') {break;}
			SingleChar[2] = String[i ++];
			SingleChar[3] = '\0';
		}
		else if ((String[i] & 0xF8) == 0xF0)	//第一个字节为11110xxx
		{
			CharLength = 4;						//字符为4字节
			SingleChar[0] = String[i ++];
			if (String[i] == '\0') {break;}
			SingleChar[1] = String[i ++];
			if (String[i] == '\0') {break;}
			SingleChar[2] = String[i ++];
			if (String[i] == '\0') {break;}
			SingleChar[3] = String[i ++];
			SingleChar[4] = '\0';
		}
		else
		{
			i ++;			//意外情况，i指向下一个字节，忽略此字节，继续判断下一个字节
			continue;
		}
#endif
		
#ifdef LCD_CHARSET_GB2312						//定义字符集为GB2312
		/*此段代码的目的是，提取GB2312字符串中的一个字符，转存到SingleChar子字符串中*/
		/*判断GB2312字节的最高位标志位*/
		if ((String[i] & 0x80) == 0x00)			//最高位为0
		{
			CharLength = 1;						//字符为1字节
			SingleChar[0] = String[i ++];		//将第一个字节写入SingleChar第0个位置，随后i指向下一个字节
			SingleChar[1] = '\0';				//为SingleChar添加字符串结束标志位
		}
		else									//最高位为1
		{
			CharLength = 2;						//字符为2字节
			SingleChar[0] = String[i ++];		//将第一个字节写入SingleChar第0个位置，随后i指向下一个字节
			if (String[i] == '\0') {break;}		//意外情况，跳出循环，结束显示
			SingleChar[1] = String[i ++];		//将第二个字节写入SingleChar第1个位置，随后i指向下一个字节
			SingleChar[2] = '\0';				//为SingleChar添加字符串结束标志位
		}
#endif
		
		/*显示上述代码提取到的SingleChar*/
		if (CharLength == 1)	//如果是单字节字符
		{
			/*使用OLED_ShowChar显示此字符*/
			LCD_ShowChar(X + XOffset, Y, SingleChar[0], fc, bc, sizey, mode);
			XOffset += sizey / 2;
		}
		else					//否则，即多字节字符
		{
			if(sizey == LCD_6X12)		//给定字体为6*12点阵
			{
				/*遍历整个字模库，从字模库中寻找此字符的数据*/
				for(pIndex = 0; strcmp(LCD_tfont12[pIndex].Index, "") != 0; pIndex ++)
				{
					/*找到匹配的字符*/
					if (strcmp(LCD_tfont12[pIndex].Index, SingleChar) == 0)
					{
						break;		//跳出循环，此时pIndex的值为指定字符的索引
					}
				}
				LCD_ShowChinese12x12(X + XOffset, Y, pIndex, fc, bc, LCD_6X12, mode);
				XOffset += sizey;
			}
			else if(sizey == LCD_8X16)		//给定字体为8*16点阵
			{
				/*遍历整个字模库，从字模库中寻找此字符的数据*/
				for(pIndex = 0; strcmp(LCD_tfont16[pIndex].Index, "") != 0; pIndex ++)
				{
					/*找到匹配的字符*/
					if (strcmp(LCD_tfont16[pIndex].Index, SingleChar) == 0)
					{
						break;		//跳出循环，此时pIndex的值为指定字符的索引
					}
				}
				LCD_ShowChinese16x16(X + XOffset, Y, pIndex, fc, bc, LCD_8X16, mode);
				XOffset += sizey;
			}
			else if(sizey == LCD_12X24)		//给定字体为12*24点阵
			{
				/*遍历整个字模库，从字模库中寻找此字符的数据*/
				for(pIndex = 0; strcmp(LCD_tfont24[pIndex].Index, "") != 0; pIndex ++)
				{
					/*找到匹配的字符*/
					if (strcmp(LCD_tfont24[pIndex].Index, SingleChar) == 0)
					{
						break;		//跳出循环，此时pIndex的值为指定字符的索引
					}
				}
				LCD_ShowChinese24x24(X + XOffset, Y, pIndex, fc, bc, LCD_12X24, mode);
				XOffset += sizey;
			}
			else if(sizey == LCD_16X32)		//给定字体为12*24点阵
			{
				/*遍历整个字模库，从字模库中寻找此字符的数据*/
				for(pIndex = 0; strcmp(LCD_tfont32[pIndex].Index, "") != 0; pIndex ++)
				{
					/*找到匹配的字符*/
					if (strcmp(LCD_tfont32[pIndex].Index, SingleChar) == 0)
					{
						break;		//跳出循环，此时pIndex的值为指定字符的索引
					}
				}
				LCD_ShowChinese32x32(X + XOffset, Y, pIndex, fc, bc, LCD_16X32, mode);
				XOffset += sizey;
			}			
			else
			{
				return;
			}
		}
	}
}

/**
  * 函    数:LCD使用printf函数打印格式化字符串（支持ASCII码和中文混合写入）
  * 参    数 x,y:显示坐标
  * 参    数 fc:字的颜色
  * 参    数 bc:字的背景色
  * 参    数：FontSize 指定字体大小
  * 参    数 mode:0非叠加模式  1叠加模式
  * 参    数：format 指定要显示的格式化字符串，范围：ASCII码可见字符或中文字符组成的字符串
  * 参    数：... 格式化字符串参数列表
  * 返 回 值：无
  * 说    明：
  */
void LCD_Printf(int16_t X, int16_t Y, uint16_t fc, uint16_t bc, uint8_t FontSize, uint8_t mode, char *format, ...)
{
	char String[256];						//定义字符数组
	va_list arg;							//定义可变参数列表数据类型的变量arg
	va_start(arg, format);					//从format开始，接收参数列表到arg变量
	vsprintf(String, format, arg);			//使用vsprintf打印格式化字符串和参数列表到字符数组中
	va_end(arg);							//结束变量arg
	LCD_ShowString(X, Y, String, fc, bc, FontSize, mode);//LCD显示字符数组（字符串）
}

/**
  * 函    数:乘方函数
  * 参    数 m,n:底数，指数
  * 返 回 值：无
  * 说    明：无
  */
uint32_t mypow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;
	return result;
}

/**
  * 函    数:LCD显示整数变量
  * 参    数 x,y:显示坐标
  * 参    数 num:要显示整数变量
  * 参    数 len:要显示的位数
  * 参    数 fc:字的颜色
  * 参    数 bc:字的背景色
  * 参    数 sizey:字号
  * 返 回 值：无
  * 说    明：无
  */
void LCD_ShowIntNum(uint16_t x,uint16_t y,uint16_t num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey)
{         	
	uint8_t t,temp;
	uint8_t enshow=0;
	uint8_t sizex=sizey/2;
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+t*sizex,y,' ',fc,bc,sizey,0);
				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
} 

/**
  * 函    数:LCD显示两位小数变量
  * 参    数 x,y:显示坐标
  * 参    数 num:要显示小数变量
  * 参    数 len:要显示的位数
  * 参    数 fc:字的颜色
  * 参    数 bc:字的背景色
  * 参    数 sizey:字号
  * 返 回 值：无
  * 说    明：无
  */
void LCD_ShowFloatNum(uint16_t x,uint16_t y,float num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey)
{         	
	uint8_t t,temp,sizex;
	uint16_t num1;
	sizex=sizey/2;
	num1=num*100;
	for(t=0;t<len;t++)
	{
		temp=(num1/mypow(10,len-t-1))%10;
		if(t==(len-2))
		{
			LCD_ShowChar(x+(len-2)*sizex,y,'.',fc,bc,sizey,0);
			t++;
			len+=1;
		}
	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
}

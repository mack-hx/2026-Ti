#ifndef __LCD_DATA_H
#define __LCD_DATA_H

#include <stdint.h>

/*字符集定义*/
/*以下两个宏定义只可解除其中一个的注释*/
#define LCD_CHARSET_UTF8			//定义字符集为UTF8
//#define LCD_CHARSET_GB2312		//定义字符集为GB2312

/*字模基本单元*/
typedef struct 
{
#ifdef LCD_CHARSET_UTF8			//定义字符集为UTF8
	char Index[5];					//汉字索引，空间为5字节
#endif
	
#ifdef LCD_CHARSET_GB2312			//定义字符集为GB2312
	char Index[3];					//汉字索引，空间为3字节
#endif
	char Data[24];
}LCD_typFNT12; 

typedef struct 
{
#ifdef LCD_CHARSET_UTF8			//定义字符集为UTF8
	char Index[5];					//汉字索引，空间为5字节
#endif
	
#ifdef LCD_CHARSET_GB2312			//定义字符集为GB2312
	char Index[3];					//汉字索引，空间为3字节
#endif
	
	uint8_t Data[32];				//字模数据
} LCD_typFNT16;

typedef struct 
{
#ifdef LCD_CHARSET_UTF8			//定义字符集为UTF8
	char Index[5];					//汉字索引，空间为5字节
#endif
	
#ifdef LCD_CHARSET_GB2312			//定义字符集为GB2312
	char Index[3];					//汉字索引，空间为3字节
#endif
	unsigned char Msk[72];
}LCD_typFNT24; 

typedef struct 
{
#ifdef LCD_CHARSET_UTF8			//定义字符集为UTF8
	char Index[5];					//汉字索引，空间为5字节
#endif
	
#ifdef LCD_CHARSET_GB2312			//定义字符集为GB2312
	char Index[3];					//汉字索引，空间为3字节
#endif
	unsigned char Msk[128];
}LCD_typFNT32; 

/*ASCII字模数据声明*/
extern const unsigned char LCD_F6X12[][12];
extern const unsigned char LCD_F8X16[][16];
extern const unsigned char LCD_F12X24[][48];
extern const unsigned char LCD_F16X32[][64];


///*汉字字模数据声明*/
extern const LCD_typFNT12 LCD_tfont12[];
extern const LCD_typFNT16 LCD_tfont16[];
extern const LCD_typFNT24 LCD_tfont24[];
extern const LCD_typFNT32 LCD_tfont32[];

///*图像数据声明*/
extern const unsigned char gImage_1[3200];
/*按照上面的格式，在这个位置加入新的图像数据声明*/
//...

#endif


/*****************江协科技|版权所有****************/
/*****************jiangxiekeji.com*****************/

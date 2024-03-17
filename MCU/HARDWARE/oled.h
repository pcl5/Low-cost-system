#ifndef __OLED_H
#define __OLED_H			  	 
#include "sys.h"
#include "system.h"

//Oled port macro definition
//OLED端口宏定义
#define OLED_RST_Clr() PDout(12)=0   //RST
#define OLED_RST_Set() PDout(12)=1   //RST

#define OLED_RS_Clr()  PDout(11)=0   //DC
#define OLED_RS_Set()  PDout(11)=1   //DC

#define OLED_SCLK_Clr()  PDout(14)=0   //SCL
#define OLED_SCLK_Set()  PDout(14)=1   //SCL

#define OLED_SDIN_Clr()  PDout(13)=0   //SDA
#define OLED_SDIN_Set()  PDout(13)=1   //SDA
#define OLED_CMD  0	//Command //写命令
#define OLED_DATA 1	//Data //写数据

//Oled control function
//OLED控制用函数
void OLED_WR_Byte(u8 dat,u8 cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);		   				   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode);
void OLED_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size);
void OLED_ShowString(u8 x,u8 y,const u8 *p);

#define CNSizeWidth  16
#define CNSizeHeight 16
//extern char Hzk16[][16];
void OLED_ShowCHinese(u8 x,u8 y,u8 no,u8 font_width,u8 font_height);	
void OLED_Set_Pos(unsigned char x, unsigned char y);
#endif  
	 

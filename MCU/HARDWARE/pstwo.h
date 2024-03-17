#ifndef __PSTWO_H
#define __PSTWO_H
#include "delay.h"
#include "sys.h"
#include "system.h"
      
#define PS2_TASK_PRIO		4     //Task priority //任务优先级
#define PS2_STK_SIZE 		256   //Task stack size //任务堆栈大小


#define DI   PEin(15)     //Input pin //输入引脚

#define DO_H PEout(12)=1   //Command height //命令位高
#define DO_L PEout(12)=0   //Command low //命令位低

#define CS_H PEout(10)=1  //Cs pull up //CS拉高
#define CS_L PEout(10)=0  //Cs drawdown //CS拉低

#define CLK_H PEout(8)=1 //Clock lift //时钟拉高
#define CLK_L PEout(8)=0 //Clock down //时钟拉低

//These are our button constants
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_GREEN       13
#define PSB_RED         14
#define PSB_BLUE        15
#define PSB_PINK        16

#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      16

#define PSS_RX 5   //Right rocker x shaft data   //右摇杆X轴数据
#define PSS_RY 6   //The left rocker y axis data //左摇杆Y轴数据
#define PSS_LX 7   //Right rocker x axis data    //右摇杆X轴数据
#define PSS_LY 8   //The left rocker y axis data //左摇杆Y轴数据

extern u8 Data[9];
extern u16 MASK[16];
extern u16 Handkey;
void PS2_Read(void);
void PS2_Init(void);
u8 PS2_RedLight(void);   
void PS2_ReadData(void); 
void PS2_Cmd(u8 CMD);		 
u8 PS2_DataKey(void);		 
u8 PS2_AnologData(u8 button); 
void PS2_ClearData(void);	    
void PS2_Vibration(u8 motor1, u8 motor2);

void PS2_EnterConfing(void);	   
void PS2_TurnOnAnalogMode(void); 
void PS2_VibrationMode(void);  
void PS2_ExitConfing(void);	 
void PS2_SetInit(void);		    
void PS2_Receive (void);
void pstwo_task(void *pvParameters);
#endif






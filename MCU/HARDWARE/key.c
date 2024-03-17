#include "key.h"

/**************************************************************************
Function: Key initialization
Input   : none
Output  : none
函数功能：按键初始化
入口参数：无
返回  值：无 
**************************************************************************/
void KEY_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIOB时钟
  GPIO_InitStructure.GPIO_Pin = KEY_PIN; //KEY对应引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIOB14	
} 
/**************************************************************************
Function: Buttons to scan
Input   : none
Output  : none
函数功能：按键扫描
入口参数：无
返回  值：按键状态 0：无动作 1：单击 
**************************************************************************/
u8 click(void)
{
	//Press the release sign
	//按键按松开标志
	static u8 flag_key=1;
	
	if(flag_key&&KEY==0)
	{
	 flag_key=0; //The key is pressed //按键按下
	 return 1;	
	}
	else if(1==KEY)			
		flag_key=1;
	return 0; //No key is pressed //无按键按下
}
/**************************************************************************
Function: Delay function
Input   : none
Output  : none
函数功能：延迟函数
入口参数：无
返 回 值：无
**************************************************************************/
void Delay_ms(void)
{
   int ii,i;    
   for(ii=0;ii<50;ii++)
   {
	   for(i=0;i<50;i++);
	 }	
}
/**************************************************************************
Function: Buttons to scan
Input   : Double click wait time
Output  : Button status: 0- no action, 1- click, 2- double click
函数功能：按键扫描
入口参数：双击等待时间
返回  值：按键状态: 0-无动作, 1-单击, 2-双击 
**************************************************************************/
u8 click_N_Double (u8 time)
{
		static	u8 flag_key,count_key,double_key;	
		static	u16 count_single,Forever_count;
	
	  if(KEY==0)  Forever_count++;   
    else        Forever_count=0;
	
		if(0==KEY&&0==flag_key)		flag_key=1;	
	  if(0==count_key)
		{
				if(flag_key==1) 
				{
					double_key++;
					count_key=1;	
				}
				if(double_key==2) 
				{
					double_key=0;
					count_single=0;
					return 2; //Double click //双击
				}
		}
		if(1==KEY)			flag_key=0,count_key=0;
		
		if(1==double_key)
		{
			count_single++;
			if(count_single>time&&Forever_count<time)
			{
			double_key=0;
			count_single=0;	
			return 1; //Click //单击
			}
			if(Forever_count>time)
			{
			double_key=0;
			count_single=0;	
			}
		}	
		return 0;
}
/**************************************************************************
Function: Button scan.Because static variables are used, a function with a different name needs to be defined when the keystroke scan function is used multiple times
Input   : none
Output  : Button status: 0- no action, 1- click, 2- double click
函数功能：按键扫描。因为使用到了静态变量，当多处需要使用按键扫描函数时，需要再定义一个不同名函数
入口参数：无
返 回 值：按键状态: 0-无动作, 1-单击, 2-双击 
**************************************************************************/
u8 click_N_Double_MPU6050 (u8 time)
{
		static	u8 flag_key,count_key,double_key;	
		static	u16 count_single,Forever_count;
	
	  if(KEY==0)  Forever_count++;  
    else        Forever_count=0;
		if(0==KEY&&0==flag_key)		flag_key=1;	
	  if(0==count_key)
		{
				if(flag_key==1) 
				{
					double_key++;
					count_key=1;	
				}
				if(double_key==2) 
				{
					double_key=0;
					count_single=0;
					return 2; //Double click //双击
				}
		}
		if(1==KEY)			flag_key=0,count_key=0;
		
		if(1==double_key)
		{
			count_single++;
			if(count_single>time&&Forever_count<time)
			{
			double_key=0;
			count_single=0;	
			return 1; //Click //单击
			}
			if(Forever_count>time)
			{
			double_key=0;
			count_single=0;	
			}
		}	
		return 0;
}
/**************************************************************************
Function: Long according to the test
Input   : none
Output  : Key state 0: no action 1: long press 3s
函数功能：长按检测
入口参数：无
返回  值：按键状态 0：无动作 1：长按3s
**************************************************************************/
u8 Long_Press(void)
{
	static u16 Long_Press_count,Long_Press;

	if(Long_Press==0&&KEY==0)  Long_Press_count++; 
	else                       Long_Press_count=0;

	if(Long_Press_count>15)	//3 seconds //3秒	
	{
		Long_Press=1;	
		Long_Press_count=0;
		return 1;
	}				
	 if(Long_Press==1) //Long press position 1 //长按标志位置1
	{
			Long_Press=0;
	}
	return 0;
}

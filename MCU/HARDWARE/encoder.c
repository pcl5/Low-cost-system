#include "encoder.h"

/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : none
Output  : none
函数功能：把TIM2初始化为编码器接口模式
入口参数：无
返 回 值：无
**************************************************************************/
void Encoder_Init_TIM2(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);   //使能定时器
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB, ENABLE);  //使用A B口
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;  //PA15
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;  //PB3
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);  

  GPIO_PinAFConfig(GPIOA,GPIO_PinSource15,GPIO_AF_TIM2);   //复用为TIM2 编码器接口
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_TIM2);   //复用为TIM2 编码器接口
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; 							// No prescaling     //不分频
  TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;  //设定计数器自动重装值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //选择时钟分频：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数    
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);  //初始化定时器
  
  TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 0;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);
  
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);//清除TIM的更新标志位
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_SetCounter(TIM2,0);
  TIM_Cmd(TIM2, ENABLE); 
}

/**************************************************************************
Function: Initialize TIM3 as the encoder interface mode
Input   : none
Output  : none
函数功能：把TIM3初始化为编码器接口模式
入口参数：无
返回  值：无
**************************************************************************/
void Encoder_Init_TIM3(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);   //使能定时器
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  //使用A口

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;  //PB4 PB5
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);  

  GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_TIM3);   //复用为TIM2 编码器接口
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_TIM3);   //复用为TIM2 编码器接口
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);  
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; 							// No prescaling     //不分频
  TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;  //设定计数器自动重装值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //选择时钟分频：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数    
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);  //初始化定时器
  
  TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 0;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);  
	
  TIM_ClearFlag(TIM3, TIM_FLAG_Update);//清除TIM的更新标志位
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_SetCounter(TIM3,0);
  TIM_Cmd(TIM3, ENABLE); 
}
/**************************************************************************
Function: Initialize TIM4 as the encoder interface mode
Input   : none
Output  : none
函数功能：把TIM4初始化为编码器接口模式
入口参数：无
返 回 值：无
**************************************************************************/
void Encoder_Init_TIM4(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//使能定时器4的时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能PB端口时钟
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;//端口配置
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);   //根据设定参数初始化GPIOB

  GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4); //复用为TIM4 编码器接口
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_TIM4); //复用为TIM4 编码器接口
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // No prescaling 
  TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //设定计数器自动重装值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   //TIM向上计数  
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  
  TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 0;
  TIM_ICInit(TIM4, &TIM_ICInitStructure);
  
  TIM_ClearFlag(TIM4, TIM_FLAG_Update);
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	TIM_SetCounter(TIM4,0);
  TIM_Cmd(TIM4, ENABLE); 
}
/**************************************************************************
Function: Initialize TIM5 as the encoder interface mode
Input   : none
Output  : none
函数功能：把TIM5初始化为编码器接口模式
入口参数：无
返回  值：无
**************************************************************************/
void Encoder_Init_TIM5(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);//使能定时器5的时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能PA端口时钟
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;//端口配置
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);   //根据设定参数初始化GPIOA

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM5); //复用为TIM5 编码器接口
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5); //复用为TIM5 编码器接口
	
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // No prescaling 
  TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //设定计数器自动重装值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   //TIM向上计数  
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
  
  TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 0;
  TIM_ICInit(TIM5, &TIM_ICInitStructure);
  
  TIM_ClearFlag(TIM5, TIM_FLAG_Update);
  TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	TIM_SetCounter(TIM5,0);
  TIM_Cmd(TIM5, ENABLE); 
}

/**************************************************************************
Function: Read the encoder count
Input   : The timer
Output  : Encoder value (representing speed)
函数功能：读取编码器计数
入口参数：定时器
返回  值：编码器数值(代表速度)
**************************************************************************/
int Read_Encoder(u8 TIMX)
{
 int Encoder_TIM;    
 switch(TIMX)
 {
	case 2:  Encoder_TIM= (short)TIM2 -> CNT;   TIM2 -> CNT=0;  break;
	case 3:  Encoder_TIM= (short)TIM3 -> CNT;   TIM3 -> CNT=0;  break;
	case 4:  Encoder_TIM= (short)TIM4 -> CNT;   TIM4 -> CNT=0;  break;	
	case 5:  Encoder_TIM= (short)TIM5 -> CNT;   TIM5 -> CNT=0;  break;	
	default: Encoder_TIM=0;
 }
	return Encoder_TIM;
}

/**************************************************************************
Function: Tim2 interrupt service function
Input   : none
Output  : none
函数功能：TIM2中断服务函数
入口参数：无
返 回 值：无
**************************************************************************/
void TIM2_IRQHandler(void)
{ 		    		  			    
	if(TIM2->SR&0X0001) //Overflow interrupt //溢出中断
	{    				   				     	    	
	}				   
	TIM2->SR&=~(1<<0); //Clear the interrupt flag bit //清除中断标志位 	    
}
/**************************************************************************
Function: Tim3 interrupt service function
Input   : none
Output  : none
函数功能：TIM3中断服务函数
入口参数：无
返 回 值：无
**************************************************************************/
void TIM3_IRQHandler(void)
{ 		    		  			    
	if(TIM3->SR&0X0001) //Overflow interrupt //溢出中断
	{    				   				     	    	
	}				   
	TIM3->SR&=~(1<<0); //Clear the interrupt flag bit //清除中断标志位  	    
}
/**************************************************************************
Function: Tim4 interrupt service function
Input   : none
Output  : none
函数功能：TIM4中断服务函数
入口参数：无
返 回 值：无
**************************************************************************/
void TIM4_IRQHandler(void)
{ 		    		  			    
	if(TIM4->SR&0X0001) //Overflow interrupt //溢出中断
	{    				   				     	    	
	}				   
	TIM4->SR&=~(1<<0); //Clear the interrupt flag bit //清除中断标志位  	    
}

/**************************************************************************
Function: Tim9 interrupt service function
Input   : none
Output  : none
函数功能：TIM9中断服务函数
入口参数：无
返 回 值：无
**************************************************************************/
void TIM5_IRQHandler(void)
{ 		    		  			    
	if(TIM5->SR&0X0001) //Overflow interrupt //溢出中断
	{    				   				     	    	
	}				   
	TIM5->SR&=~(1<<0); //Clear the interrupt flag bit //清除中断标志位  	    
}

void TIM8_BRK_TIM12_IRQHandler(void)
{
	if(TIM12->SR&0X0001) //Overflow interrupt //溢出中断
	{    				   				     	    	
	}				   
	TIM12->SR&=~(1<<0); //Clear the interrupt flag bit //清除中断标志位  	 

}

#include "control.h"	
#include "filter.h"	
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/

u8 Flag_Target,Flag_Change;                             //相关标志位
u8 temp1;                                               //临时变量
float Voltage_Count,Voltage_All; 											  //电压采样相关变量
float Gyro_K=0.004;     				  											//陀螺仪比例系数
int j;
#define a_PARAMETER          (0.275f)   
#define T 0.320f    //0.145f
#define L 0.315f    //0.17f
#define K 570.8f
/**************************************************************************
函数功能：小车运动数学模型
入口参数：Y轴速度和角度
返回  值：无
**************************************************************************/
void Kinematic_Analysis(float Vy,float angle)
{
        Target_A   = Vy*(1+T*tan(angle)/2/L);
        Target_B   = Vy*(1-T*tan(angle)/2/L);
	      Servo=SERVO_INIT-angle*K;
}
/**************************************************************************
函数功能：所有的控制代码都在这里面
         5ms定时中断由MPU6050的INT引脚触发
         严格保证采样和数据处理的时间同步				 
**************************************************************************/
int EXTI15_10_IRQHandler(void) 
{    
	 if(INT==0)		
	{     
		  EXTI->PR=1<<15;                                                      //清除LINE5上的中断标志位  		
		   Flag_Target=!Flag_Target;
		  if(delay_flag==1)
			 {
				 if(++delay_50==10)	 delay_50=0,delay_flag=0;                     //给主函数提供50ms的精准延时
			 }
		  if(Flag_Target==1)                                                  //5ms读取一次陀螺仪和加速度计的值
			{
					if(Usart_Flag==0&&PS2_ON_Flag==0&&Usart_ON_Flag==1)  memcpy(rxbuf,Urxbuf,8*sizeof(u8));	//如果解锁了串口控制标志位，进入串口控制模式
					Read_DMP();                                                           //===更新姿态		
			  	Key();//扫描按键变化	
			    return 0;	                                               
			}                                                                     	 //===10ms控制一次，为了保证M法测速的时间基准，首先读取编码器数据
			UA_Encoder=Read_Encoder(2);                                          //===读取编码器的值		
			Encoder_A=UA_Encoder/25;
			Position_A+=Encoder_A;                                                 //===积分得到速度   
			UB_Encoder=-Read_Encoder(3);                                          //===读取编码器的值		
			Encoder_B=UB_Encoder/25;
			Position_B+=Encoder_B;                                                 //===积分得到速度   
	  	Read_DMP();                                                            	//===更新姿态	
  		Led_Flash(100);                                                       	 //===LED闪烁;常规模式 1s改变一次指示灯的状态	
			Voltage_All+=Get_battery_volt();                                      	 //多次采样累积
			if(++Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//求平均值 获取电池电压	
		  if(PS2_KEY==4)PS2_ON_Flag=1,CAN_ON_Flag=0,Usart_ON_Flag=0;									
		  if(CAN_ON_Flag==1||Usart_ON_Flag==1||PS2_ON_Flag==1) CAN_N_Usart_Control();    //接到串口或者CAN遥控解锁指令之后，使能CAN和串口控制输入
			if(RC_Velocity>0&&RC_Velocity<15)  RC_Velocity=15;                   //避免电机进入低速非线性区
			if(Turn_Off(Voltage)==0)               //===如果电池电压不存在异常
			 { 			 	
				 if(CAN_ON_Flag==0&&Usart_ON_Flag==0&&PS2_ON_Flag==0)      Get_RC(Run_Flag);//===串口和CAN控制都未使能，则接收蓝牙遥控指
				 Motor_A=Incremental_PI_A(Encoder_A,Target_A);                         //===速度闭环控制计算电机A最终PWM
				 Motor_B=Incremental_PI_B(Encoder_B,Target_B);                         //===速度闭环控制计算电机B最终PWM
				 Xianfu_Pwm(6900);                     //===PWM限幅
				 Set_Pwm(-Motor_A,-Motor_B,Servo);     //===赋值给PWM寄存器  
			 }
			 else	Set_Pwm(0,0,SERVO_INIT);    //===赋值给PWM寄存器 
	 }
	 return 0;	 
} 


/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int servo)
{
	  
   	if(motor_a<0)			INA2=1,			INA1=0;
			else 	          INA2=0,			INA1=1;
		PWMA=myabs(motor_a);
	
		if(motor_b<0)			INB2=1,			INB1=0;
		else 	            INB2=0,			INB1=1;
		PWMB=myabs(motor_b);
	    SERVO=servo;
	
}

/**************************************************************************
函数功能：限制PWM赋值 
入口参数：幅值
返回  值：无
**************************************************************************/
void Xianfu_Pwm(int amplitude)
{	
    if(Motor_A<-amplitude) Motor_A=-amplitude;	
		if(Motor_A>amplitude)  Motor_A=amplitude;	
	  if(Motor_B<-amplitude) Motor_B=-amplitude;	
		if(Motor_B>amplitude)  Motor_B=amplitude;
    if (Servo>1930)	 Servo=1930;
//	  if (Servo<1035)	 Servo=1035;	
}
/**************************************************************************
函数功能：位置PID控制过程中速度的设置
入口参数：无、幅值
返回  值：无
**************************************************************************/
void Xianfu_Velocity(int amplitude_A,int amplitude_B,int amplitude_C,int amplitude_D)
{	
    if(Motor_A<-amplitude_A) Motor_A=-amplitude_A;	//位置控制模式中，A电机的运行速度
		if(Motor_A>amplitude_A)  Motor_A=amplitude_A;	  //位置控制模式中，A电机的运行速度
	  if(Motor_B<-amplitude_B) Motor_B=-amplitude_B;	//位置控制模式中，B电机的运行速度
		if(Motor_B>amplitude_B)  Motor_B=amplitude_B;		//位置控制模式中，B电机的运行速度
}
/**************************************************************************
函数功能：按键修改小车运行状态 
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{	
	u8 tmp;
	tmp=click_N_Double(100); 
	if(tmp==2)Flag_Show=!Flag_Show;//双击控制显示模式                  
}

/**************************************************************************
函数功能：异常关闭电机
入口参数：电压
返回  值：1：异常  0：正常
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<1110)//电池电压过低关闭电机
			{	                                                
      temp=1;      
      PWMA=0;
      PWMB=0;					
      }
			else
      temp=0;
      return temp;			
}

/**************************************************************************
函数功能：绝对值函数
入口参数：long int
返回  值：unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //保存上一次偏差 
	 return Pwm;                         //增量输出
}
int Incremental_PI_B (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //保存上一次偏差 
	 return Pwm;                         //增量输出
}

/**************************************************************************
函数功能：通过串口指令对小车进行遥控
入口参数：串口指令
返回  值：无
**************************************************************************/
void Get_RC(u8 mode)
{
//	  float step=0.3f;  //设置速度控制步进值。
	  if(mode==0)//速度
		{	
				 switch(Flag_Direction)   //方向控制
				 { 
				 case 1:      Move_Y=RC_Velocity;  	 	 Angle=0;        break;
				 case 2:      Move_Y=RC_Velocity;  	 	 Angle=PI/4;   	 break;
				 case 3:      Move_Y=0;      				 	 Angle=0;   	   break;
				 case 4:      Move_Y=-RC_Velocity;  	 Angle=-PI/4;    break;
				 case 5:      Move_Y=-RC_Velocity;  	 Angle=0;        break;
				 case 6:      Move_Y=-RC_Velocity;  	 Angle=PI/4;     break;
				 case 7:      Move_Y=0;     	 			 	 Angle=0;        break;
				 case 8:      Move_Y=+RC_Velocity; 	 	 Angle=-PI/4;    break; 
				 default:     Move_Y=0;                Angle=0;        break;
			 } 
//				if(Move_Y<-RC_Velocity) Move_Y=-RC_Velocity;	
//				if(Move_Y>RC_Velocity)  Move_Y=RC_Velocity;	 
//				if(Angle<-RC_Velocity) Angle=-RC_Velocity;	
//				if(Angle>RC_Velocity)  Angle=RC_Velocity;	 
	 }
		 Kinematic_Analysis(Move_Y,Angle);//得到控制目标值，进行运动学分析
}

/**************************************************************************
函数功能：接收CAN或者串口控制指令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void CAN_N_Usart_Control(void)
{
//			int flag_1, flag_2;
			int RX,LY;
		  int Yuzhi=20;
			if(CAN_ON_Flag==1||Usart_ON_Flag==1) 
			{
//			 if(rxbuf[1])flag_1=1;  else flag_1=-1;  //方向控制位
//			 if(rxbuf[3])flag_2=1;  else flag_2=-1;  //方向控制位
//			 Target_A=flag_1*rxbuf[0];
//			 Target_B=flag_2*rxbuf[1];
			 if(rxbuf[1]==0)Move_Y=rxbuf[0]; //识别运动方向
			 else           Move_Y=-rxbuf[0]; //速度
			 Angle=(rxbuf[2]-90)*PI/180;   //角度获取
			}
			else if (PS2_ON_Flag==1)
	    {
	     RX=PS2_RX-128;
			 LY=PS2_LY-128;
			 if(RX>-Yuzhi&&RX<Yuzhi)RX=0; //消除非线性区
			 if(LY>-Yuzhi&&LY<Yuzhi)LY=0;
		   Angle= RX*PI/4/120;
		   Move_Y=-LY/2.84;	 
		  //if(Move_Y<0)Angle=-Angle;	  
	    }
			Kinematic_Analysis(Move_Y,Angle);//得到控制目标值，进行运动学分析
}

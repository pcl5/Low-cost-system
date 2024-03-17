#include "robot_select_init.h"

//Initialize the robot parameter structure
//初始化机器人参数结构体
Robot_Parament_InitTypeDef  Robot_Parament; 
/**************************************************************************
Function: According to the potentiometer switch needs to control the car type
Input   : none
Output  : none
函数功能：根据电位器切换需要控制的小车类型
入口参数：无
返回  值：无
**************************************************************************/
void Robot_Select(void)
{
	//The ADC value is variable in segments, depending on the number of car models. Currently there are 6 car models, CAR_NUMBER=6
  //ADC值分段变量，取决于小车型号数量，目前有6种小车型号，CAR_NUMBER=6
	Divisor_Mode=2048/CAR_NUMBER+5;
	Car_Mode=(int) ((Get_adc_Average(Potentiometer,10))/Divisor_Mode); //Collect the pin information of potentiometer //采集电位器引脚信息	
  if(Car_Mode>5)Car_Mode=5;
	//强制
	//Car_Mode=4;
	
	switch(Car_Mode)
	{
		case Mec_Car:       Robot_Init(MEC_wheelspacing,         MEC_axlespacing,          0,                     HALL_30F, Photoelectric_500, Mecanum_75);            break; //麦克纳姆轮小车
		case Omni_Car:      Robot_Init(0,                        0,                        Omni_Turn_Radiaus_109, HALL_30F, Photoelectric_500, FullDirecion_60);       break; //全向轮小车
		case Akm_Car:       Robot_Init(Akm_wheelspacing,         Akm_axlespacing,          0,                     HALL_30F, Photoelectric_500, Black_WheelDiameter);   break; //阿克曼小车
		case Diff_Car:      Robot_Init(Diff_wheelSpacing,        0,                        0,                     HALL_30F, Photoelectric_500, Black_WheelDiameter);   break; //两轮差速小车
		case FourWheel_Car: Robot_Init(Four_Mortor_wheelSpacing, Four_Mortor__axlespacing, 0,                     HALL_30F, Photoelectric_500, Black_WheelDiameter);   break; //四驱车 
		case Tank_Car:      Robot_Init(Tank_wheelSpacing,        0,                        0,                     HALL_30F, Photoelectric_500, Tank_WheelDiameter);    break; //履带车
	}
	
	
	//Check the parameters//自检相关参数
	switch(Car_Mode)
  {
	 case Mec_Car:       CheckPhrase1=8, CheckPhrase2=14; break; //麦克纳姆轮小车
	 case Omni_Car:      CheckPhrase1=6, CheckPhrase2=10; break; //全向轮小车
	 case Akm_Car:       CheckPhrase1=4, CheckPhrase2=7;  break; //阿克曼小车
	 case Diff_Car:      CheckPhrase1=4, CheckPhrase2=7;  break; //两轮差速小车
	 case FourWheel_Car: CheckPhrase1=8, CheckPhrase2=11; break; //四驱车 
	 case Tank_Car:      CheckPhrase1=4, CheckPhrase2=7;  break; //履带车
  }
}

/**************************************************************************
Function: Initialize cart parameters
Input   : wheelspacing, axlespacing, omni_rotation_radiaus, motor_gear_ratio, Number_of_encoder_lines, tyre_diameter
Output  : none
函数功能：初始化小车参数
入口参数：轮距 轴距 自转半径 电机减速比 电机编码器精度 轮胎直径
返回  值：无
**************************************************************************/
void Robot_Init(double wheelspacing, float axlespacing, float omni_turn_radiaus, float gearratio,float Accuracy,float tyre_diameter) // 
{
	//wheelspacing, Mec_Car is half wheelspacing
	//轮距 麦轮车为半轮距
  Robot_Parament.WheelSpacing=wheelspacing; 
	//axlespacing, Mec_Car is half axlespacing
  //轴距 麦轮车为半轴距	
  Robot_Parament.AxleSpacing=axlespacing;   
	//Rotation radius of omnidirectional trolley
  //全向轮小车旋转半径		
  Robot_Parament.OmniTurnRadiaus=omni_turn_radiaus; 
	//motor_gear_ratio
	//电机减速比
  Robot_Parament.GearRatio=gearratio; 
	//Number_of_encoder_lines
  //编码器精度(编码器线数)	
  Robot_Parament.EncoderAccuracy=Accuracy;
	//Diameter of driving wheel
  //主动轮直径	
  Robot_Parament.WheelDiameter=tyre_diameter;       
	
	//Encoder value corresponding to 1 turn of motor (wheel)
	//电机(车轮)转1圈对应的编码器数值
	Encoder_precision=EncoderMultiples*Robot_Parament.EncoderAccuracy*Robot_Parament.GearRatio;
	//Driving wheel circumference
  //主动轮周长	
	Wheel_perimeter=Robot_Parament.WheelDiameter*PI;
	//wheelspacing, Mec_Car is half wheelspacing
  //轮距 麦轮车为半轮距  
  Wheel_spacing=Robot_Parament.WheelSpacing; 
  //axlespacing, Mec_Car is half axlespacing	
  //轴距 麦轮车为半轴距	
	Axle_spacing=Robot_Parament.AxleSpacing; 
	//Rotation radius of omnidirectional trolley
  //全向轮小车旋转半径	
	Omni_turn_radiaus=Robot_Parament.OmniTurnRadiaus; 
}



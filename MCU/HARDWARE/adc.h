#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
#include "system.h"
#define Battery_Ch    8 //Battery voltage, ADC channel 8 //电池电压，ADC通道8
#define Potentiometer 9  //Potentiometer, ADC channel 9 //电位器，ADC通道9
void Adc_Init(void);
void  Adc_POWER_Init(void);
u16 Get_Adc(u8 ch);
u16 Get_Adc2(u8 ch);
float Get_battery_volt(void) ;
u16 Get_adc_Average(u8 chn, u8 times);
extern float Voltage,Voltage_Count,Voltage_All; 	
#endif 



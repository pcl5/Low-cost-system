#include "adc.h"

float Voltage,Voltage_Count,Voltage_All; //Variables related to battery voltage sampling //��ص�ѹ������صı���  
const float Revise=0.99;

/**************************************************************************
Function: ADC initializes battery voltage detection
Input   : none
Output  : none
�������ܣ�ADC��ʼ����ص�ѹ���
��ڲ�������
����  ֵ����
**************************************************************************/
void  Adc_Init(void)
{  
		GPIO_InitTypeDef       GPIO_InitStructure;
		ADC_CommonInitTypeDef ADC_CommonInitStructure;
		ADC_InitTypeDef       ADC_InitStructure;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOAʱ��
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //ʹ��ADC1ʱ��


		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;//PB0 ͨ��8
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ������
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//����������
		GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��  

		RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1��λ
		RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//��λ����	 

		ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//����ģʽ
		ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//���������׶�֮����ӳ�5��ʱ��
		ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMAʧ��
		ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div6;//Ԥ��Ƶ4��Ƶ��ADCCLK=PCLK2/4=84/4=21Mhz,ADCʱ����ò�Ҫ����36Mhz 
		ADC_CommonInit(&ADC_CommonInitStructure);//��ʼ��

		ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12λģʽ
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;//��ɨ��ģʽ	
		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//�ر�����ת��
		ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//��ֹ������⣬ʹ���������
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�Ҷ���	
		ADC_InitStructure.ADC_NbrOfConversion = 1;//1��ת���ڹ��������� Ҳ����ֻת����������1 
		ADC_Init(ADC1, &ADC_InitStructure);//ADC��ʼ��
		ADC_Cmd(ADC1, ENABLE);//����ADת����	 
}		


void  Adc_POWER_Init(void)
{  
		GPIO_InitTypeDef       GPIO_InitStructure;
		ADC_CommonInitTypeDef ADC_CommonInitStructure;
		ADC_InitTypeDef       ADC_InitStructure;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOAʱ��
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE); //ʹ��ADC1ʱ��


		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//PB0 ͨ��8
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ������
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//����������
		GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��  

		RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2,ENABLE);	  //ADC2��λ
		RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2,DISABLE);	//��λ����	 

		ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//����ģʽ
		ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//���������׶�֮����ӳ�5��ʱ��
		ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMAʧ��
		ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div6;//Ԥ��Ƶ4��Ƶ��ADCCLK=PCLK2/4=84/4=21Mhz,ADCʱ����ò�Ҫ����36Mhz 
		ADC_CommonInit(&ADC_CommonInitStructure);//��ʼ��

		ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12λģʽ
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;//��ɨ��ģʽ	
		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//�ر�����ת��
		ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//��ֹ������⣬ʹ���������
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�Ҷ���	
		ADC_InitStructure.ADC_NbrOfConversion = 1;//1��ת���ڹ��������� Ҳ����ֻת����������1 
		ADC_Init(ADC2, &ADC_InitStructure);//ADC��ʼ��
		ADC_Cmd(ADC2, ENABLE);//����ADת����	 
}		
/**************************************************************************
Function: The AD sampling
Input   : The ADC channels
Output  : AD conversion results
�������ܣ�AD����
��ڲ�����ADC��ͨ��
����  ֵ��ADת�����
**************************************************************************/
u16 Get_Adc(u8 ch)   
{
	//Sets the specified ADC rule group channel, one sequence, and sampling time
	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	
	//ADC1,ADCͨ��,����ʱ��Ϊ480����
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );
  //Enable the specified ADC1 software transformation startup function	
  //ʹ��ָ����ADC1�����ת����������	
	ADC_SoftwareStartConv(ADC1);
	//Wait for the conversion to finish
  //�ȴ�ת������	
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));
	//Returns the result of the last ADC1 rule group conversion
	//�������һ��ADC1�������ת�����
	return ADC_GetConversionValue(ADC1);	
}

u16 Get_Adc2(u8 ch)   
{
	//Sets the specified ADC rule group channel, one sequence, and sampling time
	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	
	//ADC2,ADCͨ��,����ʱ��Ϊ480����
	ADC_RegularChannelConfig(ADC2, ch, 1, ADC_SampleTime_480Cycles );
  //Enable the specified ADC2 software transformation startup function	
  //ʹ��ָ����ADC1�����ת����������	
	ADC_SoftwareStartConv(ADC2);
	//Wait for the conversion to finish
  //�ȴ�ת������	
	while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC ));
	//Returns the result of the last ADC2 rule group conversion
	//�������һ��ADC1�������ת�����
	return ADC_GetConversionValue(ADC2);	
}

/**************************************************************************
Function: Collect multiple ADC values to calculate the average function
Input   : ADC channels and collection times
Output  : AD conversion results
�������ܣ��ɼ����ADCֵ��ƽ��ֵ����
��ڲ�����ADCͨ���Ͳɼ�����
�� �� ֵ��ADת�����
**************************************************************************/
u16 Get_adc_Average(u8 chn, u8 times)
{
  u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(chn);
		delay_ms(5);
	}
	return temp_val/times;
}

/**************************************************************************
Function: Read the battery voltage
Input   : none
Output  : Battery voltage in mV
�������ܣ���ȡ��ص�ѹ 
��ڲ�������
����  ֵ����ص�ѹ����λmv
**************************************************************************/
float Get_battery_volt(void)   
{  
	float Volt;
	
	//The resistance partial voltage can be obtained by simple analysis according to the schematic diagram
	//�����ѹ���������ԭ��ͼ�򵥷������Եõ�	
	Volt=Get_Adc2(Battery_Ch)*3.3*11.0*Revise/1.0/4096;	
	return Volt;
}





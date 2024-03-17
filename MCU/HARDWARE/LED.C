#include "led.h"

int Led_Count=500; //LED flicker time control //LED��˸ʱ�����

/**************************************************************************
Function: LED interface initialization
Input   : none
Output  : none
�������ܣ�LED�ӿڳ�ʼ��
��ڲ������� 
����  ֵ����
**************************************************************************/
void LED_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOBʱ��
  GPIO_InitStructure.GPIO_Pin =  LED_PIN;//LED��ӦIO��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIO
	GPIO_SetBits(GPIOA,GPIO_Pin_12);
}
/**************************************************************************
Function: Buzzer interface initialized
Input   : none
Output  : none
�������ܣ��������ӿڳ�ʼ��
��ڲ������� 
����  ֵ����
**************************************************************************/
void Buzzer_Init(void)
{	
	GPIO_InitTypeDef  GPIO_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOBʱ��
  GPIO_InitStructure.GPIO_Pin =  Buzzer_PIN;//LED��ӦIO��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIO
}
/**************************************************************************
Function: LED light flashing task
Input   : none
Output  : none
�������ܣ�LED����˸����
��ڲ������� 
����  ֵ����
**************************************************************************/
void led_task(void *pvParameters)
{
    while(1)
    {
			//The status of the LED is reversed. 0 is on and 1 is off
			//LED״̬ȡ����0�ǵ�����1��Ϩ��    
      LED=~LED;              
      //The LED flicker task is very simple, requires low frequency accuracy, and uses the relative delay function	
      //LED��˸����ǳ��򵥣���Ƶ�ʾ���Ҫ��ͣ�ʹ�������ʱ����			
      vTaskDelay(Led_Count); 
    }
}  

/**************************************************************************
Function: The LED flashing
Input   : none
Output  : blink time
�������ܣ�LED��˸
��ڲ�������˸ʱ��
�� �� ֵ����
**************************************************************************/
void Led_Flash(u16 time)
{
	  static int temp;
	  if(0==time) LED=0;
	  else		if(++temp==time)	LED=~LED,temp=0;
}


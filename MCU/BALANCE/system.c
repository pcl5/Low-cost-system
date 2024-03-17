/***********************************************
��˾����Ȥ�Ƽ�����ݸ�����޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com 
����ͨ: https://minibalance.aliexpress.com/store/4455017
�汾��V5.0
�޸�ʱ�䣺2022-05-05

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V5.0
Update��2022-05-05

All rights reserved
***********************************************/

#include "system.h"

//Robot software fails to flag bits
//���������ʧ�ܱ�־λ
u8 Flag_Stop=1;   

//The ADC value is variable in segments, depending on the number of car models. Currently there are 6 car models
//ADCֵ�ֶα�����ȡ����С���ͺ�������Ŀǰ��6��С���ͺ�
int Divisor_Mode;

// Robot type variable
//�������ͺű���
//0=Mec_Car��1=Omni_Car��2=Akm_Car��3=Diff_Car��4=FourWheel_Car��5=Tank_Car
u8 Car_Mode=0; 

//Servo control PWM value, Ackerman car special
//�������PWMֵ��������С��ר��
int Servo;  

//Default speed of remote control car, unit: mm/s
//ң��С����Ĭ���ٶȣ���λ��mm/s
float RC_Velocity=500; 

//Vehicle three-axis target moving speed, unit: m/s
//С������Ŀ���˶��ٶȣ���λ��m/s
float Move_X, Move_Y, Move_Z;   

//PID parameters of Speed control
//�ٶȿ���PID����
float Velocity_KP=300,Velocity_KI=300; 

//Smooth control of intermediate variables, dedicated to omni-directional moving cars
//ƽ�������м������ȫ���ƶ�С��ר��
Smooth_Control smooth_control;  

//The parameter structure of the motor
//����Ĳ����ṹ��
Motor_parameter MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;  

/************ С���ͺ���ر��� **************************/
/************ Variables related to car model ************/
//Encoder accuracy
//����������
float Encoder_precision; 
//Wheel circumference, unit: m
//�����ܳ�����λ��m
float Wheel_perimeter; 
//Drive wheel base, unit: m
//�������־࣬��λ��m
float Wheel_spacing; 
//The wheelbase of the front and rear axles of the trolley, unit: m
//С��ǰ�������࣬��λ��m
float Axle_spacing; 
//All-directional wheel turning radius, unit: m
//ȫ����ת��뾶����λ��m
float Omni_turn_radiaus; 
/************ С���ͺ���ر��� **************************/
/************ Variables related to car model ************/

//PS2 controller, Bluetooth APP, aircraft model controller, CAN communication, serial port 1, serial port 5 communication control flag bit.
//These 6 flag bits are all 0 by default, representing the serial port 3 control mode
//PS2�ֱ�������APP����ģ�ֱ���CANͨ�š�����1������5ͨ�ſ��Ʊ�־λ����6����־λĬ�϶�Ϊ0��������3����ģʽ
u8 PS2_ON_Flag=0, APP_ON_Flag=0, Remote_ON_Flag=0, CAN_ON_Flag=0, Usart1_ON_Flag, Usart5_ON_Flag; 

//Bluetooth remote control associated flag bits
//����ң����صı�־λ
u8 Flag_Left, Flag_Right, Flag_Direction=0, Turn_Flag; 

//Sends the parameter's flag bit to the Bluetooth APP
//������APP���Ͳ����ı�־λ
u8 PID_Send; 

//The PS2 gamepad controls related variables
//PS2�ֱ�������ر���
float PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY; 

//Self-check the relevant flag variables
//�Լ���ر�־����
int Check=0, Checking=0, Checked=0, CheckCount=0, CheckPhrase1=0, CheckPhrase2=0; 

//Check the result code
//�Լ�������
long int ErrorCode=0; 

void systemInit(void)
{       
	
//	//Interrupt priority group setti  ng
//	//�ж����ȼ���������
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
//	
//	//Delay function initialization
//	//��ʱ������ʼ��
	delay_init(168);	
	
	//Initialize the hardware interface connected to the LED lamp
	//��ʼ����LED�����ӵ�Ӳ���ӿ�
	LED_Init();                     
	    
  //Initialize the hardware interface connected to the buzzer	
  //��ʼ������������ӵ�Ӳ���ӿ�
	Buzzer_Init();  
	
	//Initialize the hardware interface connected to the enable switch
	//��ʼ����ʹ�ܿ������ӵ�Ӳ���ӿ�
	Enable_Pin();

  //Initialize the hardware interface connected to the OLED display
  //��ʼ����OLED��ʾ�����ӵ�Ӳ���ӿ�	
	OLED_Init();     
	
	//Initialize the hardware interface connected to the user's key
	//��ʼ�����û��������ӵ�Ӳ���ӿ�
	KEY_Init();	
	
	//Serial port 1 initialization, communication baud rate 115200, 
	//can be used to communicate with ROS terminal
	//����1��ʼ����ͨ�Ų�����115200����������ROS��ͨ��
	uart1_init(115200);	  
	
	//Serial port 2 initialization, communication baud rate 9600, 
	//used to communicate with Bluetooth APP terminal
	//����2��ʼ����ͨ�Ų�����9600������������APP��ͨ��
	uart2_init(9600);  
	
	//Serial port 3 is initialized and the baud rate is 115200. 
	//Serial port 3 is the default port used to communicate with ROS terminal
	//����3��ʼ����ͨ�Ų�����115200������3ΪĬ��������ROS��ͨ�ŵĴ���
	uart3_init(115200);
	
	//Serial port 5 initialization, communication baud rate 115200, 
	//can be used to communicate with ROS terminal
	//����5��ʼ����ͨ�Ų�����115200����������ROS��ͨ��
	uart5_init(115200);

	//ADC pin initialization, used to read the battery voltage and potentiometer gear, 
	//potentiometer gear determines the car after the boot of the car model
	//ADC���ų�ʼ�������ڶ�ȡ��ص�ѹ���λ����λ����λ����λ����С���������С�������ͺ�
 	Adc_Init();  
	Adc_POWER_Init();
	
	//Initialize the CAN communication interface
  //CANͨ�Žӿڳ�ʼ��
	CAN1_Mode_Init(1,7,6,3,0); 
	
  //According to the tap position of the potentiometer, determine which type of car needs to be matched, 
  //and then initialize the corresponding parameters	
  //���ݵ�λ���ĵ�λ�ж���Ҫ���������һ���ͺŵ�С����Ȼ����ж�Ӧ�Ĳ�����ʼ��	
	Robot_Select();                 
	
	 //Encoder A is initialized to read the real time speed of motor C  
  //������A��ʼ�������ڶ�ȡ���C��ʵʱ�ٶ�	
	 Encoder_Init_TIM2();
	//Encoder B is initialized to read the real time speed of motor D
  //������B��ʼ�������ڶ�ȡ���D��ʵʱ�ٶ�	
	  Encoder_Init_TIM3();   
	//Encoder C is initialized to read the real time speed of motor B
  //������C��ʼ�������ڶ�ȡ���B��ʵʱ�ٶ�	
	  Encoder_Init_TIM4(); 
	//Encoder D is initialized to read the real time speed of motor A
	//������D��ʼ�������ڶ�ȡ���A��ʵʱ�ٶ�
		Encoder_Init_TIM5(); 
	
	//��ʱ��12���������PWM�ӿ�
		TIM12_SERVO_Init(9999,84-1);  //APB1��ʱ��Ƶ��Ϊ84M , Ƶ��=84M/((9999+1)*(83+1))=100Hz
		
		//��ͨС��Ĭ�϶�ʱ��8������ģ�ӿ�
   // TIM8_SERVO_Init(9999,168-1);//APB2��ʱ��Ƶ��Ϊ168M , Ƶ��=168M/((9999+1)*(167+1))=100Hz
	 //Initialize the model remote control interface		
	 //��ʼ����ģң�ؽӿ�
	 TIM8_Cap_Init(9999,168-1);  //�߼���ʱ��TIM8��ʱ��Ƶ��Ϊ168M    

  //Initialize motor speed control and, for controlling motor speed, PWM frequency 10kHz
  //��ʼ������ٶȿ����Լ������ڿ��Ƶ���ٶȣ�PWMƵ��10KHZ
  //APB2ʱ��Ƶ��Ϊ168M����PWMΪ16799��Ƶ��=168M/((16799+1)*(0+1))=10k
		TIM1_PWM_Init(16799,0);
		TIM9_PWM_Init(16799,0);
		TIM10_PWM_Init(16799,0);
		TIM11_PWM_Init(16799,0);
		
  //IIC initialization for MPU6050 
  //IIC��ʼ��������MPU6050  
	I2C_GPIOInit();

  //MPU6050  is initialized to read the vehicle's three-axis attitude, 
	//three-axis angular velocity and three-axis acceleration information
  //MPU6050 ��ʼ�������ڶ�ȡС��������̬��������ٶȡ�������ٶ���Ϣ
   MPU6050_initialize();        		
	
	//Initialize the hardware interface to the PS2 controller
	//��ʼ����PS2�ֱ����ӵ�Ӳ���ӿ�
	PS2_Init();
	
	//PS2 gamepad configuration is initialized and configured in analog mode
  //PS2�ֱ����ó�ʼ��,����Ϊģ����ģʽ	
	PS2_SetInit();		 							
}

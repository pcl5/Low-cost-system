#include "I2C.h"
/**************************************************************************
Function: IIC pin initialization
Input   : none
Output  : none
函数功能：IIC引脚初始化
入口参数：无
返回  值：无
**************************************************************************/
void I2C_GPIOInit(void)
{
	
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化

	IIC_SCL=1;
	IIC_SDA=1;

}

/**************************************************************************
Function: Simulate IIC start signal
Input   : none
Output  : none
函数功能：模拟IIC起始信号
入口参数：无
返回  值：无
**************************************************************************/
void I2C_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;
	if(!READ_SDA)return ;	
	IIC_SCL=1;
	delay_us(1);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	if(READ_SDA)return ;
	delay_us(1);
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
	return ;
}

/**************************************************************************
Function: Analog IIC end signal
Input   : none
Output  : none
函数功能：模拟IIC结束信号
入口参数：无
返回  值：无
**************************************************************************/
void I2C_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(1);
	IIC_SCL=1; 
	IIC_SDA=1;//发送I2C总线结束信号
	delay_us(1);	
}



bool I2C_WaiteForAck(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC_SDA=1;
	delay_us(1);	   
	IIC_SCL=1;
	delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			I2C_Stop();
			return 0;
		}
	  delay_us(1);
	}
	IIC_SCL=0;//时钟输出0 	   
	return 1;
}

/**************************************************************************
Function: IIC response
Input   : none
Output  : none
函数功能：IIC应答
入口参数：无
返回  值：无
**************************************************************************/
void I2C_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}

/**************************************************************************
Function: IIC don't reply
Input   : none
Output  : none
函数功能：IIC不应答
入口参数：无
返回  值：无
**************************************************************************/ 
void I2C_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}



bool I2C_WriteOneBit(uint8_t DevAddr, uint8_t RegAddr, uint8_t BitNum, uint8_t Data)
{
    uint8_t Dat;
    
    Dat =I2C_ReadOneByte(DevAddr, RegAddr);
    Dat = (Data != 0) ? (Dat | (1 << BitNum)) : (Dat & ~(1 << BitNum));
    I2C_WriteOneByte(DevAddr, RegAddr, Dat);
    
    return true;
}




bool I2C_WriteBits(uint8_t DevAddr, uint8_t RegAddr, uint8_t BitStart, uint8_t Length, uint8_t Data)
{

    uint8_t Dat, Mask;
    
	Dat = I2C_ReadOneByte(DevAddr, RegAddr);
    Mask = (0xFF << (BitStart + 1)) | 0xFF >> ((8 - BitStart) + Length - 1);
    Data <<= (8 - Length);
    Data >>= (7 - BitStart);
    Dat &= Mask;
    Dat |= Data;
    I2C_WriteOneByte(DevAddr, RegAddr, Dat);
    
    return true;
}

/**************************************************************************
Function: IIC sends a bit
Input   : none
Output  : none
函数功能：IIC发送一个位
入口参数：无
返回  值：无
**************************************************************************/
void I2C_WriteByte(uint8_t Data)
{
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(Data&0x80)>>7;
        Data<<=1; 	  
		delay_us(1);   
		IIC_SCL=1;
		delay_us(1); 
		IIC_SCL=0;	
		delay_us(1);
    }	 
}


u8 I2C_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data)
{
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
	I2C_WaiteForAck();
	I2C_WriteByte(RegAddr);
	I2C_WaiteForAck();
	I2C_WriteByte(Data);
	I2C_WaiteForAck();
	I2C_Stop();
	return 1;
}


bool I2C_WriteBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff)
{
	uint8_t i;

	if(0 == Num || NULL == pBuff)
	{
		return false;
	}
	
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
	I2C_WaiteForAck();
	I2C_WriteByte(RegAddr);
	I2C_WaiteForAck();
	
	for(i = 0; i < Num; i ++)
	{
		I2C_WriteByte(*(pBuff + i));
		I2C_WaiteForAck();
	}
	I2C_Stop();

	return true;
}

/**************************************************************************
Function: IIC reads a bit
Input   : none
Output  : none
函数功能：IIC读取一个位
入口参数：无
返回  值：无
**************************************************************************/
uint8_t I2C_ReadByte(uint8_t Ack)
{
	uint8_t i, RecDat = 0;

	SDA_IN();
	for(i = 0; i < 8; i ++)
	{
	//	I2C_SCL_Clr();
		IIC_SCL=0;
		delay_us(1);
//		I2C_SCL_Set();
				IIC_SCL=1;
		RecDat <<= 1;
		if(READ_SDA)
			RecDat |= 0x01;
		else
			RecDat &= ~0x01;
		delay_us(1);
	}
	if(I2C_ACK == Ack)
		I2C_Ack();
	else
		I2C_NAck();

	return RecDat;
}




uint8_t I2C_ReadOneByte(uint8_t DevAddr, uint8_t RegAddr)
{
	uint8_t TempVal = 0;
	
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
	I2C_WaiteForAck();
	I2C_WriteByte(RegAddr);
	I2C_WaiteForAck();
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Receiver);
	I2C_WaiteForAck();
	TempVal = I2C_ReadByte(I2C_NACK);
	I2C_Stop();
	
	return TempVal;
}

bool I2C_ReadBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff)
{
	uint8_t i;

	if(0 == Num || NULL == pBuff)
	{
		return false;
	}
	
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
	I2C_WaiteForAck();
	I2C_WriteByte(RegAddr);
	I2C_WaiteForAck();
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Receiver);
	I2C_WaiteForAck();

	for(i = 0; i < Num; i ++)
	{
		if((Num - 1) == i)
		{
			*(pBuff + i) = I2C_ReadByte(I2C_NACK);
		}
		else
		{
			*(pBuff + i) = I2C_ReadByte(I2C_ACK);
		}
	}

	I2C_Stop();
	
	return true;
}


///**************************************************************************
//Function: IIC continuous reading data
//Input   : dev：Target device IIC address；reg:Register address；
//					length：Number of bytes；*data:The pointer where the read data will be stored
//Output  : count：Number of bytes read out-1
//函数功能：IIC连续读数据
//入口参数：dev：目标设备IIC地址；reg:寄存器地址；length：字节数；
//					*data:读出的数据将要存放的指针
//返回  值：count：读出来的字节数量-1
//**************************************************************************/ 
//u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data){
//    u8 count = 0;
//	
//	IIC_Start();
//	IIC_Send_Byte(dev);	   //发送写命令
//	IIC_Wait_Ack();
//	IIC_Send_Byte(reg);   //发送地址
//  IIC_Wait_Ack();	  
//	IIC_Start();
//	IIC_Send_Byte(dev+1);  //进入接收模式	
//	IIC_Wait_Ack();
//	
//    for(count=0;count<length;count++){
//		 
//		 if(count!=length-1)   data[count]=IIC_Read_Byte(1);  //带ACK的读数据
//		 else                  data[count]=IIC_Read_Byte(0);  //最后一个字节NACK
//	}
//    IIC_Stop();//产生一个停止条件
//    return count;
//}




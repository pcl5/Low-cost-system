#include "dma.h"
#include "sys.h"

DMA_InitTypeDef DMA_InitStructure;
u16 DMA1_MEM_LEN2;//保存DMA每次数据传送的长度

/**************************************************************************
函数功能：串口2DMA初始化
入口参数：DMA通道，数据的目标地址，数据的起始地址，发送的数据数量
返回  值：无
**************************************************************************/
void MYDMA_Init2(DMA_Channel_TypeDef* DMA_CHx,u32 cpar,u32 cmar,u16 cndtr)
{
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//使能DMA传输
	
  DMA_DeInit(DMA_CHx);	//将DMA的通道1寄存器重设为缺省值
	DMA1_MEM_LEN2=cndtr;		//数据传输量 
	DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;  //DMA外设ADC基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = cmar;  //DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //数据传输方向，从内存读取发送到外设
	DMA_InitStructure.DMA_BufferSize = cndtr;  //DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //工作在正常缓存模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; //DMA通道 x拥有高优先级 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA_CHx, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道USART1_Tx_DMA_Channel所标识的寄存器 	
} 
/**************************************************************************
函数功能：开启一次DMA传输
入口参数：DMA通道，数据的目标地址，数据的起始地址，发送的数据数量
返回  值：无
**************************************************************************/
void MYDMA_Enable2(DMA_Channel_TypeDef*DMA_CHx)
{ 
	DMA_Cmd(DMA_CHx, DISABLE );  //关闭USART1 TX DMA1 所指示的通道      
 	DMA_SetCurrDataCounter(DMA1_Channel7,DMA1_MEM_LEN2);//DMA通道的DMA缓存的大小
 	DMA_Cmd(DMA_CHx, ENABLE);  //使能USART1 TX DMA1 所指示的通道 
}	  

///**************************************************************************
//函数功能：DMA方式的printf
//入口参数：要打印的内容
//返回  值：无
//**************************************************************************/
void DMA_printf(const char *format,...)
{
	u32 length;
	va_list args;
	
	va_start(args, format);
	length = vsnprintf((char*)t2xbuf, sizeof(t2xbuf), (char*)format, args);
    va_end(args);
	USART_SendBuffer((const char*)t2xbuf,length);
}

u32 USART_SendBuffer(const char* buffer, u32 length)
{
	if( (buffer==NULL) || (length==0) )
	{
		return 0;
	}
 
	DMA_Cmd(DMA1_Channel7, DISABLE); //失能DMA通道
	DMA_SetCurrDataCounter(DMA1_Channel7, length);
	DMA_Cmd(DMA1_Channel7, ENABLE); //使能DMA通道
	while(1)
	{
		if(DMA_GetITStatus(DMA1_IT_TC7)!=RESET)	//判断通道7（uart2_tx）传输完成
		{
			DMA_ClearFlag(DMA1_IT_TC7);//清除通道7传输完成标志
			break;
		}
	}
	return length;
}

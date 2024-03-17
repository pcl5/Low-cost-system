#include "dma.h"
#include "sys.h"

DMA_InitTypeDef DMA_InitStructure;
u16 DMA1_MEM_LEN2;//����DMAÿ�����ݴ��͵ĳ���

/**************************************************************************
�������ܣ�����2DMA��ʼ��
��ڲ�����DMAͨ�������ݵ�Ŀ���ַ�����ݵ���ʼ��ַ�����͵���������
����  ֵ����
**************************************************************************/
void MYDMA_Init2(DMA_Channel_TypeDef* DMA_CHx,u32 cpar,u32 cmar,u16 cndtr)
{
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//ʹ��DMA����
	
  DMA_DeInit(DMA_CHx);	//��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ
	DMA1_MEM_LEN2=cndtr;		//���ݴ����� 
	DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;  //DMA����ADC����ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = cmar;  //DMA�ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //���ݴ��䷽�򣬴��ڴ��ȡ���͵�����
	DMA_InitStructure.DMA_BufferSize = cndtr;  //DMAͨ����DMA����Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //�����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //��������������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; //DMAͨ�� xӵ�и����ȼ� 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
	DMA_Init(DMA_CHx, &DMA_InitStructure);  //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��USART1_Tx_DMA_Channel����ʶ�ļĴ��� 	
} 
/**************************************************************************
�������ܣ�����һ��DMA����
��ڲ�����DMAͨ�������ݵ�Ŀ���ַ�����ݵ���ʼ��ַ�����͵���������
����  ֵ����
**************************************************************************/
void MYDMA_Enable2(DMA_Channel_TypeDef*DMA_CHx)
{ 
	DMA_Cmd(DMA_CHx, DISABLE );  //�ر�USART1 TX DMA1 ��ָʾ��ͨ��      
 	DMA_SetCurrDataCounter(DMA1_Channel7,DMA1_MEM_LEN2);//DMAͨ����DMA����Ĵ�С
 	DMA_Cmd(DMA_CHx, ENABLE);  //ʹ��USART1 TX DMA1 ��ָʾ��ͨ�� 
}	  

///**************************************************************************
//�������ܣ�DMA��ʽ��printf
//��ڲ�����Ҫ��ӡ������
//����  ֵ����
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
 
	DMA_Cmd(DMA1_Channel7, DISABLE); //ʧ��DMAͨ��
	DMA_SetCurrDataCounter(DMA1_Channel7, length);
	DMA_Cmd(DMA1_Channel7, ENABLE); //ʹ��DMAͨ��
	while(1)
	{
		if(DMA_GetITStatus(DMA1_IT_TC7)!=RESET)	//�ж�ͨ��7��uart2_tx���������
		{
			DMA_ClearFlag(DMA1_IT_TC7);//���ͨ��7������ɱ�־
			break;
		}
	}
	return length;
}

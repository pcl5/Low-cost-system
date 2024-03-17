#include "can.h"
#include "system.h"
/**************************************************************************
Function: CAN1 initialization
Input   : tsjw：Resynchronize the jump time unit, Scope: 1 ~ 3;
 			    tbs2：Time unit of time period 2, range :1~8;
 			    tbs1：Time unit of time period 1, range :1~16;
 			    brp ：Baud rate divider, range :1 to 1024;(We're actually going to add 1, which is 1 to 1024) tq=(brp)*tpclk1
 			    mode：0, normal mode;1. Loop mode;
Output  : 0- Initialization successful;Other - initialization failed
Note: none of the entry parameters (except mode) can be 0
函数功能：CAN1初始化
入口参数：tsjw：重新同步跳跃时间单元，范围:1~3;
 			    tbs2：时间段2的时间单元，范围:1~8;
 			    tbs1：时间段1的时间单元，范围:1~16;
 			    brp ：波特率分频器，范围:1~1024;(实际要加1,也就是1~1024) tq=(brp)*tpclk1
 			    mode：0,普通模式;1,回环模式;
返回  值：0-初始化成功; 其他-初始化失败
注意：入口参数(除了mode)均不能为0
波特率/Baud rate=Fpclk1/((tbs1+tbs2+1)*brp)，Fpclk1为36M
                =42M/((3+2+1)*6)
						    =1M
**************************************************************************/
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	u16 i=0;
 	if(tsjw==0||tbs2==0||tbs1==0||brp==0)return 1;
	tsjw-=1; //Subtract 1 before setting //先减去1.再用于设置
	tbs2-=1;
	tbs1-=1;
	brp-=1;

	//使能相关时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能PORTB时钟	                   											 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	
	//初始化GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化PB8 PB9
//	//引脚复用映射配置
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_CAN1); //GPIOB8复用为CAN1
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_CAN1); //GPIOB9复用为CAN1

	CAN1->MCR=0x0000;	//Exit sleep mode (setting all bits to 0 at the same time) //退出睡眠模式(同时设置所有位为0)
	CAN1->MCR|=1<<0;	//Request CAN1 to enter initialization mode //请求CAN1进入初始化模式
	while((CAN1->MSR&1<<0)==0)
	{
		i++;
		if(i>100)return 2; //Failed to enter initialization mode //进入初始化模式失败
	}
	//Non-time triggered communication mode
	//非时间触发通信模式
	CAN1->MCR|=0<<7;
  //Software automatic offline management	
	//软件自动离线管理
	CAN1->MCR|=0<<6;	
	//Sleep mode is awakened by software (clear CAN1- >;
  //睡眠模式通过软件唤醒(清除CAN1->MCR的SLEEP位)	
	CAN1->MCR|=0<<5;
	//Disallow automatic message transmission
  //禁止报文自动传送	
	CAN1->MCR|=1<<4;
	//Messages are not locked, the new overwrites the old
  //报文不锁定,新的覆盖旧的	
	CAN1->MCR|=0<<3;
	//The priority is determined by the message identifier	
  //优先级由报文标识符决定  
	CAN1->MCR|=0<<2;
  //Clear the original Settings	
  //清除原来的设置	
	CAN1->BTR=0x00000000; 
	//Mode set to 0, normal mode;1. Loop mode;
	//模式设置 0,普通模式;1,回环模式;
	CAN1->BTR|=mode<<30;
  //Resynchronization jump width (TSJW) is TSJW +1 time unit	
	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位
	CAN1->BTR|=tsjw<<24; 
  //Tbs2= Tbs2 +1 time unit	
	//Tbs2=tbs2+1个时间单位
	CAN1->BTR|=tbs2<<20; 
	//Tbs1= Tbs1 +1 time unit	
  //Tbs1=tbs1+1个时间单位	
	CAN1->BTR|=tbs1<<16;
	//Frequency division coefficient (Fdiv) is brp +1, boulder rate: Fpclk1/((Tbs1+Tbs2+1)*Fdiv)
  //分频系数(Fdiv)为brp+1，波特率:Fpclk1/((Tbs1+Tbs2+1)*Fdiv)
	CAN1->BTR|=brp<<0;  
  //Request CAN1 to exit initialization mode	
  //请求CAN1退出初始化模式			
	CAN1->MCR&=~(1<<0);	  
	while((CAN1->MSR&1<<0)==1)
	{
		i++;
		if(i>0XFFF0)return 3; //Failed to exit initialization mode //退出初始化模式失败
	}
	/*** Filter initialization || 过滤器初始化 ***/
	//Filter groups work in initialization mode
	//过滤器组工作在初始化模式
	CAN1->FMR|=1<<0;	
	//Filter 0 is not active
  //过滤器0不激活	
	CAN1->FA1R&=~(1<<0);	
	//The filter bit width is 32 bits
  //过滤器位宽为32位	
	CAN1->FS1R|=1<<0; 
	//Filter 0 works in identifier masking bit mode
  //过滤器0工作在标识符屏蔽位模式	
	CAN1->FM1R|=0<<0;	
	//Filter 0 is associated with FIFO0
  //过滤器0关联到FIFO0	
	CAN1->FFA1R|=0<<0;	
  //32 bit ID	
	//32位ID
	CAN1->sFilterRegister[0].FR1=0X00000000;
	//32-bit MASK
	//32位MASK
	CAN1->sFilterRegister[0].FR2=0X00000000;
	//Activation filter 0
	//激活过滤器0
	CAN1->FA1R|=1<<0;	
	//Filter group enters normal mode
  //过滤器组进入正常模式	
	CAN1->FMR&=0<<0;			

#if CAN1_RX0_INT_ENABLE
  //Enable to interrupt reception //使能中断接收
	
	//FIFO0消息挂号中断允许
	CAN1->IER|=1<<1;			
	
	//Configure CAN to receive interrupts
	//配置CAN接收中断
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  //Preemption priority	
	//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	//Son priority
	//子优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  //IRQ channel enablement
	//IRQ通道使能	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initializes the VIC register according to the specified parameters 
	//根据指定的参数初始化VIC寄存器	
	NVIC_Init(&NVIC_InitStructure);	

  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	
#endif
	return 0;
}   
/**************************************************************************
Function: CAN sends data
Input   : id:Standard ID(11 bits)/ Extended ID(11 bits +18 bits)
			    ide:0, standard frame;1, extension frames
			    rtr:0, data frame;1, remote frame
			    len:Length of data to be sent (fixed at 8 bytes, valid data is 6 bytes in time-triggered mode) 
			    *dat:Pointer to the data
Output  : 0~3, mailbox number. 0xFF, no valid mailbox
函数功能：CAN发送数据
入口参数：id:标准ID(11位)/扩展ID(11位+18位)	    
			    ide:0,标准帧;1,扩展帧
			    rtr:0,数据帧;1,远程帧
			    len:要发送的数据长度(固定为8个字节,在时间触发模式下,有效数据为6个字节)
			    *dat:数据指针.
返回  值：0~3,邮箱编号.0XFF,无有效邮箱
**************************************************************************/
u8 CAN1_Tx_Msg(u32 id,u8 ide,u8 rtr,u8 len,u8 *dat)
{	   
	u8 mbox;	  
	if(CAN1->TSR&(1<<26))mbox=0;		  //Mailbox 0 is empty //邮箱0为空
	else if(CAN1->TSR&(1<<27))mbox=1;	//Mailbox 1 is empty //邮箱1为空
	else if(CAN1->TSR&(1<<28))mbox=2;	//Mailbox 2 is empty //邮箱2为空
	else return 0XFF;					        //No empty mailbox, cannot send //无空邮箱,无法发送 
	
	CAN1->sTxMailBox[mbox].TIR=0; //Clear the previous Settings //清除之前的设置		
	if(ide==0) //The standard frame //标准帧
	{
		id&=0x7ff; //Take the low 11 bit STDID //取低11位stdid
		id<<=21;		  
	}else	//Extend the frame //扩展帧
	{
		id&=0X1FFFFFFF; //Take a low 32-bit extid //取低32位extid
		id<<=3;									   
	}
	CAN1->sTxMailBox[mbox].TIR|=id;		 
	CAN1->sTxMailBox[mbox].TIR|=ide<<2;	  
	CAN1->sTxMailBox[mbox].TIR|=rtr<<1;
	len&=0X0F; //Get lower 4 bits //得到低四位
	CAN1->sTxMailBox[mbox].TDTR&=~(0X0000000F);
	CAN1->sTxMailBox[mbox].TDTR|=len;	//Set the DLC	//设置DLC
	//The data to be sent is stored in the mailbox
	//待发送数据存入邮箱
	CAN1->sTxMailBox[mbox].TDHR=(((u32)dat[7]<<24)|
								((u32)dat[6]<<16)|
 								((u32)dat[5]<<8)|
								((u32)dat[4]));
	CAN1->sTxMailBox[mbox].TDLR=(((u32)dat[3]<<24)|
								((u32)dat[2]<<16)|
 								((u32)dat[1]<<8)|
								((u32)dat[0]));
	CAN1->sTxMailBox[mbox].TIR|=1<<0; //Request to send mailbox data//请求发送邮箱数据
	return mbox;
}
/**************************************************************************
Function: Get the send status
Input   : Mbox: mailbox number
Output  : 0, hang;0X05, send failed;0X07, successful transmission
函数功能：获得发送状态
入口参数：mbox：邮箱编号
返回  值：0,挂起;0X05,发送失败;0X07,发送成功
**************************************************************************/
u8 CAN1_Tx_Staus(u8 mbox)
{	
	u8 sta=0;					    
	switch (mbox)
	{
		case 0: 
			sta |= CAN1->TSR&(1<<0);			   //RQCP0
			sta |= CAN1->TSR&(1<<1);			   //TXOK0
			sta |=((CAN1->TSR&(1<<26))>>24); //TME0
			break;
		case 1: 
			sta |= CAN1->TSR&(1<<8)>>8;		   //RQCP1
			sta |= CAN1->TSR&(1<<9)>>8;		   //TXOK1
			sta |=((CAN1->TSR&(1<<27))>>25); //TME1	   
			break;
		case 2: 
			sta |= CAN1->TSR&(1<<16)>>16;	   //RQCP2
			sta |= CAN1->TSR&(1<<17)>>16;	   //TXOK2
			sta |=((CAN1->TSR&(1<<28))>>26); //TME2
			break;
		default:
			sta=0X05; //Wrong email number, failed //邮箱号不对,失败
		break;
	}
	return sta;
} 
/**************************************************************************
Function: Returns the number of packets received in FIFO0/FIFO1
Input   : Fifox: FIFO number (0, 1)
Output  : Number of packets in FIFO0/FIFO1
函数功能：得到在FIFO0/FIFO1中接收到的报文个数
入口参数：fifox：FIFO编号（0、1）
返回  值：FIFO0/FIFO1中的报文个数
**************************************************************************/
u8 CAN1_Msg_Pend(u8 fifox)
{
	if(fifox==0)return CAN1->RF0R&0x03; 
	else if(fifox==1)return CAN1->RF1R&0x03; 
	else return 0;
}
/**************************************************************************
Function: Receive data
Input   : fifox：Email
		    	id:Standard ID(11 bits)/ Extended ID(11 bits +18 bits)
			    ide:0, standard frame;1, extension frames 
			    rtr:0, data frame;1, remote frame
			    len:Length of data received (fixed at 8 bytes, valid at 6 bytes in time-triggered mode)
			    dat:Data cache
Output  : none
函数功能：接收数据
入口参数：fifox：邮箱号
		    	id:标准ID(11位)/扩展ID(11位+18位)
			    ide:0,标准帧;1,扩展帧
			    rtr:0,数据帧;1,远程帧
			    len:接收到的数据长度(固定为8个字节,在时间触发模式下,有效数据为6个字节)
			    dat:数据缓存区
返回  值：无 
**************************************************************************/
void CAN1_Rx_Msg(u8 fifox,u32 *id,u8 *ide,u8 *rtr,u8 *len,u8 *dat)
{	   
	*ide=CAN1->sFIFOMailBox[fifox].RIR&0x04; //Gets the value of the identifier selection bit //得到标识符选择位的值  
 	if(*ide==0) //Standard identifier //标准标识符
	{
		*id=CAN1->sFIFOMailBox[fifox].RIR>>21;
	}else	     //Extended identifier //扩展标识符
	{
		*id=CAN1->sFIFOMailBox[fifox].RIR>>3;
	}
	*rtr=CAN1->sFIFOMailBox[fifox].RIR&0x02;	//Gets the remote send request value //得到远程发送请求值
	*len=CAN1->sFIFOMailBox[fifox].RDTR&0x0F; //Get the DLC //得到DLC
 	//*fmi=(CAN1->sFIFOMailBox[FIFONumber].RDTR>>8)&0xFF; //Get the FMI //得到FMI
	//Receive data //接收数据
	dat[0]=CAN1->sFIFOMailBox[fifox].RDLR&0XFF;
	dat[1]=(CAN1->sFIFOMailBox[fifox].RDLR>>8)&0XFF;
	dat[2]=(CAN1->sFIFOMailBox[fifox].RDLR>>16)&0XFF;
	dat[3]=(CAN1->sFIFOMailBox[fifox].RDLR>>24)&0XFF;    
	dat[4]=CAN1->sFIFOMailBox[fifox].RDHR&0XFF;
	dat[5]=(CAN1->sFIFOMailBox[fifox].RDHR>>8)&0XFF;
	dat[6]=(CAN1->sFIFOMailBox[fifox].RDHR>>16)&0XFF;
	dat[7]=(CAN1->sFIFOMailBox[fifox].RDHR>>24)&0XFF;    
  if(fifox==0)CAN1->RF0R|=0X20;      //Free the FIFO0 mailbox //释放FIFO0邮箱
	else if(fifox==1)CAN1->RF1R|=0X20; //Free the FIFO1 mailbox //释放FIFO1邮箱	 
}
/**************************************************************************
Function: CAN receives interrupt service function, conditional compilation
Input   : none
Output  : none
函数功能：CAN接收中断服务函数，条件编译
入口参数：无
返回  值：无 
**************************************************************************/
#if CAN1_RX0_INT_ENABLE	//Enable RX0 interrupt //使能RX0中断	    
void CAN1_RX0_IRQHandler(void)
{
	u32 id;
	u8 ide,rtr,len;     

	u8 temp_rxbuf[8];

 	CAN1_Rx_Msg(0,&id,&ide,&rtr,&len,temp_rxbuf);
	if(id==0x181)
	{
		float Vz;
		CAN_ON_Flag=1, PS2_ON_Flag=0,Remote_ON_Flag=0,APP_ON_Flag=0,Usart1_ON_Flag=0,Usart5_ON_Flag=0;
		command_lost_count=0;
		//Calculate the three-axis target velocity, unit: m/s
		//计算三轴目标速度，单位：m/s
		Move_X=((float)((short)((temp_rxbuf[0]<<8)+(temp_rxbuf[1]))))/1000; 
		Move_Y=((float)((short)((temp_rxbuf[2]<<8)+(temp_rxbuf[3]))))/1000;
		Vz    =((float)((short)((temp_rxbuf[4]<<8)+(temp_rxbuf[5]))))/1000;
		if(Car_Mode==Akm_Car)
		{
			Move_Z=Vz_to_Akm_Angle(Move_X, Vz);
		}
		else
		{
			Move_Z=((float)((short)((temp_rxbuf[4]<<8)+(temp_rxbuf[5]))))/1000;
		}
	}
}
#endif

/**************************************************************************
Function: CAN1 sends a set of data (fixed format :ID 0X601, standard frame, data frame)
Input   : msg:Pointer to the data
    			len:Data length (up to 8)
Output  : 0, success, others, failure;
函数功能：CAN1发送一组数据(固定格式:ID为0X601,标准帧,数据帧)
入口参数：msg:数据指针
    			len:数据长度(最大为8)
返回  值：0,成功，其他,失败;
**************************************************************************/
u8 CAN1_Send_Msg(u8* msg,u8 len)
{	
	u8 mbox;
	u16 i=0;	  	 						       
  mbox=CAN1_Tx_Msg(0X601,0,0,len,msg);   
	while((CAN1_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))i++; //Waiting for the end of sending //等待发送结束
	if(i>=0XFFF)return 1; //Send failure //发送失败
	return 0;	//Send a success //发送成功									
}
/**************************************************************************
Function: The CAN1 port receives data queries
Input   : Buf: The data cache
Output  : 0, number of data received, other, length of data received
函数功能：CAN1口接收数据查询
入口参数：buf:数据缓存区
返回  值：0,无数据被收到，其他,接收的数据长度
**************************************************************************/
u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
	u32 id;
	u8 ide,rtr,len; 
	if(CAN1_Msg_Pend(0)==0)return 0;			   //No data received, exit directly //没有接收到数据,直接退出 	 
  	CAN1_Rx_Msg(0,&id,&ide,&rtr,&len,buf); //Read the data //读取数据
    if(id!=0x12||ide!=0||rtr!=0)len=0;		 //Receive error //接收错误	   
	return len;	
}
/**************************************************************************
Function: Can1 sends a set of data tests
Input   : msg:Pointer to the data
			    len:Data length (up to 8)
Output  : 0, success, 1, failure
函数功能：CAN1发送一组数据测试
入口参数：msg:数据指针
			    len:数据长度(最大为8)
返回  值：0,成功，1,失败
**************************************************************************/
u8 CAN1_Send_MsgTEST(u8* msg,u8 len)
{	
	u8 mbox;
	u16 i=0;	  	 						       
    mbox=CAN1_Tx_Msg(0X701,0,0,len,msg);   
	while((CAN1_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))i++; //Waiting for the end of sending //等待发送结束
	if(i>=0XFFF)return 1;	//Send failure //发送失败
	return 0;	//Send a success //发送成功
}
/**************************************************************************
Function: Sends an array to the given ID
Input   : id：ID no.
			    msg：The transmitted data pointer
Output  : 0, success, 1, failure
函数功能：给给定的id发送一个数组的命令
入口参数：id：ID号
			    msg：被输送数据指针
返回  值：0,成功，1,失败
**************************************************************************/
u8 CAN1_Send_Num(u32 id,u8* msg)
{
	u8 mbox;
	u16 i=0;	  	 						       
  mbox=CAN1_Tx_Msg(id,0,0,8,msg);   
	while((CAN1_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))i++; //Waiting for the end of sending //等待发送结束
	if(i>=0XFFF)return 1;	//Send failure //发送失败
	return 0;
}

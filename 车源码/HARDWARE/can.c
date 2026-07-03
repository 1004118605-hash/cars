#include "can.h"
#include "system.h"
/**************************************************************************
函数功能：CAN1初始化
入口参数：tsjw：重新同步跳跃时间单元，范围:1~3;
 			    tbs2：时间段2的时间单元，范围:1~8;
 			    tbs1：时间段1的时间单元，范围:1~16;
 			    brp ：波特率分频器，范围:1~1024;(实际要加1,也就是1~1024) tq=(brp)*tpclk1
 			    mode：0,普通模式;1,回环模式;
返回  值：0-初始化成功; 其他-初始化失败
注意：入口参数(除了mode)均不能为0
波特率/Baud rate=Fpclk1/((tbs1+tbs2+tsjw)*brp)，Fpclk1为42M
**************************************************************************/
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	u16 i=0;
 	if(tsjw==0||tbs2==0||tbs1==0||brp==0)return 1;
	tsjw-=1; //先减去1.再用于设置
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

	CAN1->MCR=0x0000;	//退出睡眠模式(同时设置所有位为0)
	CAN1->MCR|=1<<0;	//请求CAN1进入初始化模式
	while((CAN1->MSR&1<<0)==0)
	{
		i++;
		if(i>100)return 2; //进入初始化模式失败
	}
	//非时间触发通信模式
	CAN1->MCR|=0<<7;
	//软件自动离线管理
	CAN1->MCR|=0<<6;	
  //睡眠模式通过软件唤醒(清除CAN1->MCR的SLEEP位)	
	CAN1->MCR|=0<<5;
  //禁止报文自动传送	
	CAN1->MCR|=1<<4;
  //报文不锁定,新的覆盖旧的	
	CAN1->MCR|=0<<3;
  //优先级由报文标识符决定  
	CAN1->MCR|=0<<2;
  //清除原来的设置	
	CAN1->BTR=0x00000000; 
	//模式设置 0,普通模式;1,回环模式;
	CAN1->BTR|=mode<<30;
	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位
	CAN1->BTR|=tsjw<<24; 
	//Tbs2=tbs2+1个时间单位
	CAN1->BTR|=tbs2<<20; 
  //Tbs1=tbs1+1个时间单位	
	CAN1->BTR|=tbs1<<16;
  //分频系数(Fdiv)为brp+1，波特率:Fpclk1/((Tbs1+Tbs2+1)*Fdiv)
	CAN1->BTR|=brp<<0;  
  //请求CAN1退出初始化模式			
	CAN1->MCR&=~(1<<0);	  
	while((CAN1->MSR&1<<0)==1)
	{
		i++;
		if(i>0XFFF0)return 3;//退出初始化模式失败
	}
	/*** 过滤器初始化 ***/
	//过滤器组工作在初始化模式
	CAN1->FMR|=1<<0;	
  //过滤器0不激活	
	CAN1->FA1R&=~(1<<0);	
  //过滤器位宽为32位	
	CAN1->FS1R|=1<<0; 
  //过滤器0工作在标识符屏蔽位模式	
	CAN1->FM1R|=0<<0;	
  //过滤器0关联到FIFO0	
	CAN1->FFA1R|=0<<0;	
	//32位ID
	CAN1->sFilterRegister[0].FR1=0X00000000;
	//32位MASK
	CAN1->sFilterRegister[0].FR2=0X00000000;
	//激活过滤器0
	CAN1->FA1R|=1<<0;	
  //过滤器组进入正常模式	
	CAN1->FMR&=0<<0;			

#if CAN1_RX0_INT_ENABLE
  //使能中断接收
	
	//FIFO0消息挂号中断允许
	CAN1->IER|=1<<1;			
	
	//配置CAN接收中断
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	//子优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	//IRQ通道使能	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
	//根据指定的参数初始化VIC寄存器	
	NVIC_Init(&NVIC_InitStructure);	

  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	
#endif
	return 0;
}   
/**************************************************************************
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
	if(CAN1->TSR&(1<<26))mbox=0;		//邮箱0为空
	else if(CAN1->TSR&(1<<27))mbox=1;	//邮箱1为空
	else if(CAN1->TSR&(1<<28))mbox=2;	//邮箱2为空
	else return 0XFF;					//无空邮箱,无法发送 
	
	CAN1->sTxMailBox[mbox].TIR=0; //清除之前的设置		
	if(ide==0) //标准帧
	{
		id&=0x7ff; //取低11位stdid
		id<<=21;		  
	}else	//扩展帧
	{
		id&=0X1FFFFFFF; //取低32位extid
		id<<=3;									   
	}
	CAN1->sTxMailBox[mbox].TIR|=id;		 
	CAN1->sTxMailBox[mbox].TIR|=ide<<2;	  
	CAN1->sTxMailBox[mbox].TIR|=rtr<<1;
	len&=0X0F; //得到低四位
	CAN1->sTxMailBox[mbox].TDTR&=~(0X0000000F);
	CAN1->sTxMailBox[mbox].TDTR|=len;//设置DLC
	//待发送数据存入邮箱
	CAN1->sTxMailBox[mbox].TDHR=(((u32)dat[7]<<24)|
								((u32)dat[6]<<16)|
 								((u32)dat[5]<<8)|
								((u32)dat[4]));
	CAN1->sTxMailBox[mbox].TDLR=(((u32)dat[3]<<24)|
								((u32)dat[2]<<16)|
 								((u32)dat[1]<<8)|
								((u32)dat[0]));
	CAN1->sTxMailBox[mbox].TIR|=1<<0; //请求发送邮箱数据
	return mbox;
}
/**************************************************************************
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
			sta=0X05; //邮箱号不对,失败
		break;
	}
	return sta;
} 
/**************************************************************************
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
	*ide=CAN1->sFIFOMailBox[fifox].RIR&0x04;//得到标识符选择位的值  
 	if(*ide==0) //标准标识符
	{
		*id=CAN1->sFIFOMailBox[fifox].RIR>>21;
	}else	      //扩展标识符
	{
		*id=CAN1->sFIFOMailBox[fifox].RIR>>3;
	}
	*rtr=CAN1->sFIFOMailBox[fifox].RIR&0x02;	//得到远程发送请求值
	*len=CAN1->sFIFOMailBox[fifox].RDTR&0x0F;  //得到DLC
 	//*fmi=(CAN1->sFIFOMailBox[FIFONumber].RDTR>>8)&0xFF; //得到FMI
	//接收数据
	dat[0]=CAN1->sFIFOMailBox[fifox].RDLR&0XFF;
	dat[1]=(CAN1->sFIFOMailBox[fifox].RDLR>>8)&0XFF;
	dat[2]=(CAN1->sFIFOMailBox[fifox].RDLR>>16)&0XFF;
	dat[3]=(CAN1->sFIFOMailBox[fifox].RDLR>>24)&0XFF;    
	dat[4]=CAN1->sFIFOMailBox[fifox].RDHR&0XFF;
	dat[5]=(CAN1->sFIFOMailBox[fifox].RDHR>>8)&0XFF;
	dat[6]=(CAN1->sFIFOMailBox[fifox].RDHR>>16)&0XFF;
	dat[7]=(CAN1->sFIFOMailBox[fifox].RDHR>>24)&0XFF;    
  if(fifox==0)CAN1->RF0R|=0X20;      //释放FIFO0邮箱
	else if(fifox==1)CAN1->RF1R|=0X20;//释放FIFO1邮箱	 
}
/**************************************************************************
函数功能：CAN接收中断服务函数，条件编译
入口参数：无
返回  值：无 
**************************************************************************/
#if CAN1_RX0_INT_ENABLE	//使能RX0中断	    
void CAN1_RX0_IRQHandler(void)
{
	u32 id;
	u8 ide,rtr,len;     

	u8 temp_rxbuf[8];

 	CAN1_Rx_Msg(0,&id,&ide,&rtr,&len,temp_rxbuf);
	
	//回充装备特征码
	if(id==0x12345678)
	{
		u8 check=0;
		for(u8 i=0;i<8;i++) 
		{
			check += temp_rxbuf[i];
		}
		if( check==8 ) Get_Charging_HardWare=1;
	}
	
	else if(id==0x181)
	{
		float Vz;
		CAN_ON_Flag=1, PS2_ON_Flag=0,Remote_ON_Flag=0,APP_ON_Flag=0,Usart1_ON_Flag=0,Usart5_ON_Flag=0;
		command_lost_count=0;
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
	
	//自动回充相关
	else if(id==0x182)
	{
		charger_check=0;
		
		//自动回充状态调整pid参数使电机响应加快
		if(Allow_Recharge) Velocity_KP=600,Velocity_KI=600;
		else Velocity_KP=300,Velocity_KI=300;
		
		//计算三轴目标速度，单位：mm/s
		Recharge_Red_Move_X=((float)((short)((temp_rxbuf[0]<<8)+(temp_rxbuf[1]))))/1000; 
		Recharge_Red_Move_Y=((float)((short)((temp_rxbuf[2]<<8)+(temp_rxbuf[3]))))/1000;
		Recharge_Red_Move_Z=((float)((short)((temp_rxbuf[4]<<8)+(temp_rxbuf[5]))))/1000;
		
		Charging=temp_rxbuf[6]&1;		//充电状态标志位
		L_A = (temp_rxbuf[6]>>5)&0x01;
		L_B = (temp_rxbuf[6]>>4)&0x01;
		R_B = (temp_rxbuf[6]>>3)&0x01;
		R_A = (temp_rxbuf[6]>>2)&0x01;
		RED_STATE = L_A + L_B + R_B + R_A;
		
		if(RED_STATE==0) Recharge_Red_Move_X = Recharge_Red_Move_Y = Recharge_Red_Move_Z = 0;
		if( Charging==1) Recharge_Red_Move_X = Recharge_Red_Move_Y = Recharge_Red_Move_Z = 0;
		
		//充电电流换算
		if(temp_rxbuf[7]>128)Charging_Current=-(256-temp_rxbuf[7])*30;
		else Charging_Current=(temp_rxbuf[7]*30);
	}
}
#endif

/**************************************************************************
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
	while((CAN1_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))i++;  //等待发送结束
	if(i>=0XFFF)return 1; //发送失败
	return 0;	//发送成功									
}
/**************************************************************************
函数功能：CAN1口接收数据查询
入口参数：buf:数据缓存区
返回  值：0,无数据被收到，其他,接收的数据长度
**************************************************************************/
u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
	u32 id;
	u8 ide,rtr,len; 
	if(CAN1_Msg_Pend(0)==0)return 0;		 //没有接收到数据,直接退出 	 
  	CAN1_Rx_Msg(0,&id,&ide,&rtr,&len,buf); //读取数据
    if(id!=0x12||ide!=0||rtr!=0)len=0;		//接收错误	   
	return len;	
}
/**************************************************************************
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
	while((CAN1_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))i++; //等待发送结束
	if(i>=0XFFF)return 1;	 //发送失败
	return 0;	 //发送成功
}
/**************************************************************************
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
	while((CAN1_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))i++;  //等待发送结束
	if(i>=0XFFF)return 1;	 //发送失败
	return 0;
}

u8 CAN1_Send_EXTid_Num(u32 id,u8* msg)
{
	u8 mbox;
	u16 i=0;	  	 						       
	mbox=CAN1_Tx_Msg(id,1,0,8,msg);   
	while((CAN1_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))i++; //等待发送结束
	if(i>=0XFFF)return 1;	//发送失败
	return 0;
}



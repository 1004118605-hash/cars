#include "timer.h"

//通道1输入捕获标志，高两位做捕获标志，低6位做溢出标志		
u8 TIM8CH1_CAPTURE_STA = 0;	
u16 TIM8CH1_CAPTURE_UPVAL;
u16 TIM8CH1_CAPTURE_DOWNVAL;

//通道2输入捕获标志，高两位做捕获标志，低6位做溢出标志	
u8 TIM8CH2_CAPTURE_STA = 0;		
u16 TIM8CH2_CAPTURE_UPVAL;
u16 TIM8CH2_CAPTURE_DOWNVAL;

//通道3输入捕获标志，高两位做捕获标志，低6位做溢出标志	
u8 TIM8CH3_CAPTURE_STA = 0;		
u16 TIM8CH3_CAPTURE_UPVAL;
u16 TIM8CH3_CAPTURE_DOWNVAL;

//通道4输入捕获标志，高两位做捕获标志，低6位做溢出标志
u8 TIM8CH4_CAPTURE_STA = 0;			
u16 TIM8CH4_CAPTURE_UPVAL;
u16 TIM8CH4_CAPTURE_DOWNVAL;

u32 TIM8_T1;
u32 TIM8_T2;
u32 TIM8_T3;
u32 TIM8_T4;

//航模遥控采集相关变量
int Remoter_Ch1=1500,Remoter_Ch2=1500,Remoter_Ch3=1500,Remoter_Ch4=1500;
//航模遥控接收变量
int L_Remoter_Ch1=1500,L_Remoter_Ch2=1500,L_Remoter_Ch3=1500,L_Remoter_Ch4=1500;  

/**************************************************************************
函数功能：航模遥控初始化函数，定时器1输入捕获初始化
入口参数：arr：自动重装值，psc：时钟预分频数 
返 回 值：无
**************************************************************************/ 
void TIM8_Cap_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  	//TIM1时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//使能PORTE时钟	
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9; //GPIOC
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); 

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM8);

	/*** 初始化定时器1 ***/
	//设定计数器自动重装值 
	TIM_TimeBaseStructure.TIM_Period = arr; 
	//预分频器 
	TIM_TimeBaseStructure.TIM_Prescaler = psc; 	
	//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	//TIM向上计数模式	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); 

	/*** 初始化TIM1输入捕获参数，通道1  ***/
	//选择输入端 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; 
  //上升沿捕获
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	 //配置输入分频,不分频 
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	
   //配置输入滤波器
	TIM_ICInitStructure.TIM_ICFilter = 0x0F;	  
	TIM_ICInit(TIM8, &TIM_ICInitStructure);

	/*** 初始化TIM1输入捕获参数，通道2  ***/
	 //选择输入端 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	//上升沿捕获
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	//配置输入分频,不分频 
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  
	TIM_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 配置输入滤波器
	TIM_ICInit(TIM8, &TIM_ICInitStructure);

	/*** 初始化TIM1输入捕获参数，通道3  ***/
	//选择输入端 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;   
	 //上升沿捕获
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	//配置输入分频,不分频 
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  
	//配置输入滤波器，不滤波  
	TIM_ICInitStructure.TIM_ICFilter = 0x00;	  
	TIM_ICInit(TIM8, &TIM_ICInitStructure);

	/*** 初始化TIM1输入捕获参数，通道4  ***/
	//选择输入端 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4; 
	//上升沿捕获
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	 //配置输入分频,不分频 
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	   
	//配置输入滤波器，不滤波  
	TIM_ICInitStructure.TIM_ICFilter = 0x00;	  
	TIM_ICInit(TIM8, &TIM_ICInitStructure);

  /***  中断分组初始化 ***/
  //TIM1中断
	NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn; 
  //先占优先级0级	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  
	 //从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; 
	//IRQ通道被使能
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 
	NVIC_Init(&NVIC_InitStructure);   
	
  //不允许更新中断，允许CC1IE,CC2IE,CC3IE,CC4IE捕获中断	
	TIM_ITConfig(TIM8, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,	ENABLE);   
	 //高级定时器输出必须使能这句	
	TIM_CtrlPWMOutputs(TIM8,ENABLE); 	
	//使能定时器
	TIM_Cmd(TIM8, ENABLE); 		
}
/**************************************************************************
函数功能：航模遥控接收中断，即定时器8输入捕获中断
入口参数：无
返 回 值：无
**************************************************************************/ 
void TIM8_CC_IRQHandler(void)
{
	static u8 ch1_filter_times=0,ch2_filter_times=0,ch3_filter_times=0,ch4_filter_times=0;
	
	//连接航模遥遥控器后，需要推下前进杆，才可以正式航模控制小车
  if(Remoter_Ch2>1600&&Remote_ON_Flag==0&&SysVal.Time_count>=CONTROL_DELAY)
  {
		//航模遥控标志位置1，其它标志位置0
		Remote_ON_Flag=1;
	  APP_ON_Flag=0;
		PS2_ON_Flag=0;
		CAN_ON_Flag=0;
		//Usart_ON_Flag=0; 
		Usart1_ON_Flag=0;
		Usart5_ON_Flag=0;
	}
	
//通道一
	if ((TIM8CH1_CAPTURE_STA & 0X80) == 0) 			
	{
		if (TIM_GetITStatus(TIM8, TIM_IT_CC1) != RESET)  //通道1发生捕获事件
		{
			TIM_ClearITPendingBit(TIM8, TIM_IT_CC1); //清除中断标志位
			if (TIM8CH1_CAPTURE_STA & 0X40)	//捕获到一个下降沿
			{
				TIM8CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM8); //记录下此时的定时器计数值
				if (TIM8CH1_CAPTURE_DOWNVAL < TIM8CH1_CAPTURE_UPVAL)
				{
					TIM8_T1 = 9999;
				}
				else
					TIM8_T1 = 0;
				Remoter_Ch1 = TIM8CH1_CAPTURE_DOWNVAL - TIM8CH1_CAPTURE_UPVAL + TIM8_T1;	//得到总的高电平的时间
				if(abs(Remoter_Ch1-L_Remoter_Ch1)>500)
				{
					ch1_filter_times++;
					if( ch1_filter_times<=5 ) Remoter_Ch1=L_Remoter_Ch1;  //滤波	
					else ch1_filter_times=0;
				}
				else
				{
					ch1_filter_times=0;
				}
				L_Remoter_Ch1=Remoter_Ch1;
				
				TIM8CH1_CAPTURE_STA = 0;	//捕获标志位清零
				TIM_OC1PolarityConfig(TIM8, TIM_ICPolarity_Rising); //设置为上升沿捕获		  
			}
			else 
			{
				//发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
				TIM8CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM8); //获取上升沿数据
				TIM8CH1_CAPTURE_STA |= 0X40;  //标记已捕获到上升沿
				TIM_OC1PolarityConfig(TIM8, TIM_ICPolarity_Falling);  //设置为下降沿捕获
			}
		}
	}
  //通道二
	if ((TIM8CH2_CAPTURE_STA & 0X80) == 0)		
	{
		if (TIM_GetITStatus(TIM8, TIM_IT_CC2) != RESET)	 //通道2发生捕获事件
		{
			TIM_ClearITPendingBit(TIM8, TIM_IT_CC2);  //清除中断标志位
			if (TIM8CH2_CAPTURE_STA & 0X40)	 //捕获到一个下降沿
			{
				TIM8CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM8);  //记录下此时的定时器计数值
				if (TIM8CH2_CAPTURE_DOWNVAL < TIM8CH2_CAPTURE_UPVAL)
				{
					TIM8_T2 = 9999;
				}
				else
					TIM8_T2 = 0;
				Remoter_Ch2 = TIM8CH2_CAPTURE_DOWNVAL - TIM8CH2_CAPTURE_UPVAL + TIM8_T2; //得到总的高电平的时间
				if(abs(Remoter_Ch2-L_Remoter_Ch2)>500)
				{
					ch2_filter_times++;
					if( ch2_filter_times<=5 ) Remoter_Ch2=L_Remoter_Ch2; //滤波	
					else ch2_filter_times=0;
				}
				else
				{
					ch2_filter_times=0;
				}
				
				L_Remoter_Ch2=Remoter_Ch2;
				
				TIM8CH2_CAPTURE_STA = 0; //捕获标志位清零
				TIM_OC2PolarityConfig(TIM8, TIM_ICPolarity_Rising); //设置为上升沿捕获		  
			}
			else 
			{
				//发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
				TIM8CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM8); //获取上升沿数据
				TIM8CH2_CAPTURE_STA |= 0X40; //标记已捕获到上升沿
				TIM_OC2PolarityConfig(TIM8, TIM_ICPolarity_Falling);  //设置为下降沿捕获
			}
		}
	}
  //通道三
	if ((TIM8CH3_CAPTURE_STA & 0X80) == 0)			
	{
		if (TIM_GetITStatus(TIM8, TIM_IT_CC3) != RESET)	//通道3发生捕获事件
		{
			TIM_ClearITPendingBit(TIM8, TIM_IT_CC3); //清除中断标志位
			if (TIM8CH3_CAPTURE_STA & 0X40)	 //捕获到一个下降沿
			{
				TIM8CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM8); //记录下此时的定时器计数值
				if (TIM8CH3_CAPTURE_DOWNVAL < TIM8CH3_CAPTURE_UPVAL)
				{
					TIM8_T3 = 9999;
				}
				else
					TIM8_T3 = 0;
				Remoter_Ch3 = TIM8CH3_CAPTURE_DOWNVAL - TIM8CH3_CAPTURE_UPVAL + TIM8_T3; //得到总的高电平的时间
				if(abs(Remoter_Ch3-L_Remoter_Ch3)>500)
				{
					ch3_filter_times++;
					if( ch3_filter_times<=5 ) Remoter_Ch3=L_Remoter_Ch3; //滤波	
					else ch3_filter_times=0;
				}
				else
				{
					ch3_filter_times=0;
				}
				L_Remoter_Ch3=Remoter_Ch3;	
				TIM8CH3_CAPTURE_STA = 0; //捕获标志位清零
				TIM_OC3PolarityConfig(TIM8, TIM_ICPolarity_Rising);  //设置为上升沿捕获		  
			}
			else 
			{
				//发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
				TIM8CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM8); //获取上升沿数据
				TIM8CH3_CAPTURE_STA |= 0X40; //标记已捕获到上升沿
				TIM_OC3PolarityConfig(TIM8, TIM_ICPolarity_Falling); //设置为下降沿捕获
			}
		}
	}
//	

		 //通道四
		if ((TIM8CH4_CAPTURE_STA & 0X80) == 0)		
		{
			if (TIM_GetITStatus(TIM8, TIM_IT_CC4) != RESET)	//通道4发生捕获事件
			{
				TIM_ClearITPendingBit(TIM8, TIM_IT_CC4); //清除中断标志位
				if (TIM8CH4_CAPTURE_STA & 0X40)	//捕获到一个下降沿
				{
					TIM8CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM8); //记录下此时的定时器计数值
					if (TIM8CH4_CAPTURE_DOWNVAL < TIM8CH4_CAPTURE_UPVAL)
					{
						TIM8_T4 = 9999;
					}
					else
						TIM8_T4 = 0;
					Remoter_Ch4 = TIM8CH4_CAPTURE_DOWNVAL - TIM8CH4_CAPTURE_UPVAL + TIM8_T4;  //得到总的高电平的时间
					if(abs(Remoter_Ch4-L_Remoter_Ch4)>500)
					{
						ch4_filter_times++;
						if( ch4_filter_times<=5 ) Remoter_Ch4=L_Remoter_Ch4;  //滤波	
						else ch4_filter_times=0;
					}
					else
					{
						ch4_filter_times=0;
					}
					
					L_Remoter_Ch4=Remoter_Ch4;			
					TIM8CH4_CAPTURE_STA = 0; 	//捕获标志位清零
					TIM_OC4PolarityConfig(TIM8, TIM_ICPolarity_Rising); //设置为上升沿捕获		  
				}
				else 
				{

				  //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
					TIM8CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM8); //获取上升沿数据
					TIM8CH4_CAPTURE_STA |= 0X40;  //标记已捕获到上升沿
					TIM_OC4PolarityConfig(TIM8, TIM_ICPolarity_Falling);  //设置为下降沿捕获
				}
			}
		}
}
/**************************************************************************
函数功能：定时器8更新中断
入口参数：无
返回  值：无 
**************************************************************************/
void TIM8_UP_TIM13_IRQHandler(void) 
{ 
	//清除中断标志位 
  TIM8->SR&=~(1<<0);	    
}

void TIM8_SERVO_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;           //IO
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; //定时器
	TIM_OCInitTypeDef  TIM_OCInitStructure;        //PWM输出
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  	//TIM1时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//使能PORTE时钟	
		
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF; 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz; 	
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM8); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM8); 


	/***  初始化定时器1 ***/
//设定计数器自动重装值 
	TIM_TimeBaseStructure.TIM_Period = arr; 
	 //预分频器 
	TIM_TimeBaseStructure.TIM_Prescaler = psc; 	
	 //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	//TIM向上计数模式	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); 


	//-----------舵机初始化-----------//
  //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	//比较输出使能
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	//设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_Pulse = 0; 
  //输出极性:TIM输出比较极性高	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;   
	
  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx	
	TIM_OC1Init(TIM8, &TIM_OCInitStructure); 
	TIM_OC2Init(TIM8, &TIM_OCInitStructure); 
	TIM_OC3Init(TIM8, &TIM_OCInitStructure); 
	TIM_OC4Init(TIM8, &TIM_OCInitStructure); 
	//通道预装载使能	 
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
	//-----------舵机初始化-----------//
	TIM_ITConfig(TIM8, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,	DISABLE);
	TIM_CtrlPWMOutputs(TIM8,ENABLE); 	
	//使能定时器
	TIM_Cmd(TIM8, ENABLE); 		 

	//通道值初始化为1500，舵机零点对应值
//	TIM8->CCR1=1500;
//	TIM8->CCR2=1500;
//	TIM8->CCR3=1500;
//	TIM8->CCR4=1500;
}


void TIM12_SERVO_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;           //IO
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; //定时器
	TIM_OCInitTypeDef  TIM_OCInitStructure;        //PWM输出
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE);  	//TIM1时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//使能PORTE时钟	
		
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF; 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz; 	
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_TIM12); 
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_TIM12); 


	/***初始化定时器1 ***/
 //设定计数器自动重装值 
	TIM_TimeBaseStructure.TIM_Period = arr; 
//预分频器 
	TIM_TimeBaseStructure.TIM_Prescaler = psc; 	
//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
//TIM向上计数模式	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure); 


	//-----------舵机初始化-----------//
  //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	//比较输出使能
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	//设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_Pulse = 0; 	
  //输出极性:TIM输出比较极性高	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;   
	
  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx	
	TIM_OC1Init(TIM12, &TIM_OCInitStructure); 
		TIM_OC2Init(TIM12, &TIM_OCInitStructure); 
	//通道预装载使能	 
	TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);
		TIM_OC2PreloadConfig(TIM12, TIM_OCPreload_Enable);
	//-----------舵机初始化-----------//


	TIM_CtrlPWMOutputs(TIM12,ENABLE); 	
//使能定时器
	TIM_Cmd(TIM12, ENABLE); 		 

	//通道值初始化为1500，舵机零点对应值
	TIM12->CCR2=1500;
	TIM12->CCR1=1500;

}


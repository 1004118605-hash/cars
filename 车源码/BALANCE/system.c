

#include "system.h"

//机器人软件失能标志位
u8 Flag_Stop=0;


int Divisor_Mode;
u8 Car_Mode=0;
int Servo;
float RC_Velocity=500;
float Move_X, Move_Y, Move_Z;
float Velocity_KP=300,Velocity_KI=300;
Smooth_Control smooth_control;
Motor_parameter MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;
float Encoder_precision;
float Wheel_perimeter;
//主动轮轮距，单位：m
float Wheel_spacing;
//小车前后轴的轴距，单位：m
float Axle_spacing;
//全向轮转弯半径，单位：m
float Omni_turn_radiaus;
u8 PS2_ON_Flag=0, APP_ON_Flag=0, Remote_ON_Flag=0, CAN_ON_Flag=0, Usart1_ON_Flag, Usart5_ON_Flag;
//蓝牙遥控相关的标志位
u8 Flag_Left, Flag_Right, Flag_Direction=0, Turn_Flag;
//向蓝牙APP发送参数的标志位
u8 PID_Send;
//PS2手柄控制相关变量
float PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY;
//自检相关标志变量
int Check=0;
//自检结果代码
long int ErrorCode=0;

u8 Get_Charging_HardWare=0;//是否存在自动回充装备
u8 charger_check=0;//实时检测充电装备在线状态
//系统相关变量
SYS_VAL_t SysVal;
//电位器记录数值
int POT_val;
//电机满转值
int Full_rotation = 16799;
void systemInit(void)
{
	//中断优先级分组设置
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	//延时函数初始化
    delay_init(168);
    //IIC初始化，用于IMU
    I2C_GPIOInit();
	//系统相关软件参数初始化
	SYS_VAL_t_Init(&SysVal);
    //串口1初始化，通信波特率115200，可用于与ROS端通信
    uart1_init(115200);
    //串口2初始化，通信波特率9600，用于与蓝牙APP端通信
    uart2_init(9600);
    //串口3初始化，通信波特率115200，串口3为默认用于与ROS端通信的串口
    uart3_init(115200);
    //串口5初始化，通信波特率115200，可用于与ROS端通信
    uart5_init(115200);
	
	if( MPU6050_DEFAULT_ADDRESS == MPU6050_getDeviceID() )
	{
		SysVal.HardWare_Ver = V1_0;
		//初始化与PS2手柄连接的硬件接口
		PS2_Init();
		//PS2手柄配置初始化,配置为模拟量模式	
		PS2_SetInit();		 
		//初始化与LED灯连接的硬件接口
		V1_0_LED_Init(); 
		//MPU6050初始化，用于读取小车三轴角速度、三轴加速度信息
		MPU6050_initialize();
	}
	//如果IMU型号为ICM20948,则是新版C30D
	else if( REG_VAL_WIA == ICM20948_getDeviceID() )//读取ICM20948 id
	{
		SysVal.HardWare_Ver = V1_1;
		
		//USBHOS初始化
		MX_USB_HOST_Init();
		//初始化与LED灯连接的硬件接口
		V1_1_LED_Init();
		//ICM20948初始化，用于读取小车三轴角速度、三轴加速度信息
		invMSInit();
	}
	else //无法识别的陀螺仪,复位系统
	{
		NVIC_SystemReset();
	}         
    //初始化与蜂鸣器连接的硬件接口
    Buzzer_Init();
    //初始化与使能开关连接的硬件接口
    Enable_Pin();

    //初始化与OLED显示屏连接的硬件接口
    OLED_Init();
    //初始化与用户按键连接的硬件接口
    KEY_Init();
		if(KEY==0)			Check=1;
		else if(KEY==1)	Check=0;

    Adc_Init();
    Adc_POWER_Init();


	
	//CAN通信接口初始化
	//波特率计算方式：42M/[(参数1+参数2+参数3)*参数4]
	//示例：42M/[(1+7+6)*3] = 1M （只跟前4个参数有关）
    CAN1_Mode_Init(1,7,6,3,0);

   
    //根据电位器的档位判断需要适配的是哪一种型号的小车，然后进行对应的参数初始化
    Robot_Select();
    //编码器A初始化，用于读取电机C的实时速度
    Encoder_Init_TIM2();
    //编码器B初始化，用于读取电机D的实时速度
    Encoder_Init_TIM3();
    //编码器C初始化，用于读取电机B的实时速度
    Encoder_Init_TIM4();
    //编码器D初始化，用于读取电机A的实时速度
    Encoder_Init_TIM5();

    //定时器12用作舵机的PWM接口
    TIM12_SERVO_Init(9999,84-1);  //APB1的时钟频率为84M , 频率=84M/((9999+1)*(83+1))=100Hz

    //普通小车默认定时器8用作航模接口
    // TIM8_SERVO_Init(9999,168-1);//APB2的时钟频率为168M , 频率=168M/((9999+1)*(167+1))=100Hz
    //初始化航模遥控接口
    TIM8_Cap_Init(9999,168-1);  //高级定时器TIM8的时钟频率为168M
    //初始化电机速度控制以及，用于控制电机速度，PWM频率10KHZ
    //APB2时钟频率为168M，满PWM为16799，频率=168M/((16799+1)*(0+1))=10k
    TIM1_PWM_Init(16799,0);
    TIM9_PWM_Init(16799,0);
    TIM10_PWM_Init(16799,0);
    TIM11_PWM_Init(16799,0);

    //采集1次电池电压作为电压平滑滤波的初始化参考值
    base_vol = Get_battery_volt();

    //访问是否存在自动回充装备
    delay_ms(100);
    Find_Charging_HardWare();
	
	//读取Flash数据
	FlashParam_Read();
}

void Find_Charging_HardWare(void)
{
    u8 tmpdata[8]= {1,1,1,1,1,1,1,1};
    CAN1_Send_EXTid_Num(0x12345678,tmpdata);
}


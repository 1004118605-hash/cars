#include "robot_select_init.h"

//Initialize the robot parameter structure

Robot_Parament_InitTypeDef  Robot_Parament; 
void Robot_Select(void)
{

	Divisor_Mode=2048/CAR_NUMBER+5;
	Car_Mode=(int) ((Get_adc_Average(Potentiometer,10))/Divisor_Mode); //Collect the pin information of potentiometer //采集电位器引脚信息	
  if(Car_Mode>7)Car_Mode=7;

	
	switch(Car_Mode)
	{
		case Mec_Car:       Robot_Init(MEC_wheelspacing,         MEC_axlespacing,          0,                     HALL_30F, Photoelectric_500, Mecanum_75);            break; //麦克纳姆轮小车
		case Omni_Car:      Robot_Init(0,                        0,                        Omni_Turn_Radiaus_109, HALL_30F, Photoelectric_500, FullDirecion_60);       break; //全向轮小车
		case Akm_Car:       Robot_Init(Akm_wheelspacing,         Akm_axlespacing,          0,                     HALL_30F, Photoelectric_500, Black_WheelDiameter);   break; //阿克曼小车
		case Diff_Car:      Robot_Init(Diff_wheelSpacing,        0,                        0,                     HALL_30F, Photoelectric_500, Black_WheelDiameter);   break; //两轮差速小车
		case FourWheel_Car: Robot_Init(Four_Mortor_wheelSpacing, Four_Mortor__axlespacing, 0,                     HALL_30F, Photoelectric_500, Black_WheelDiameter);   break; //四驱车 
		case Tank_Car:      Robot_Init(Tank_wheelSpacing,        0,                        0,                     HALL_30F, Photoelectric_500, Tank_WheelDiameter);    break; //履带车
		case Mec_Car_V550:  Robot_Init(V550_MEC_wheelspacing,    V550_MEC_axlespacing,     0,                     HALL_30F, Photoelectric_500, Mecanum_75);            break; //V550-麦轮
		case FourWheel_Car_V550:  Robot_Init(V550_FourMortorWheelSpacing, V550_FourMortorAxlespacing,  0,         HALL_30F, Photoelectric_500, Black_WheelDiameter);   break; //V550-四驱
	}
}

/**************************************************************************
函数功能：初始化小车参数
入口参数：轮距 轴距 自转半径 电机减速比 电机编码器精度 轮胎直径
返回  值：无
**************************************************************************/
void Robot_Init(double wheelspacing, float axlespacing, float omni_turn_radiaus, float gearratio,float Accuracy,float tyre_diameter) // 
{
	
  Robot_Parament.WheelSpacing=wheelspacing; 
  Robot_Parament.AxleSpacing=axlespacing;   
  Robot_Parament.OmniTurnRadiaus=omni_turn_radiaus; 
  Robot_Parament.GearRatio=gearratio; 
  Robot_Parament.EncoderAccuracy=Accuracy;	
  Robot_Parament.WheelDiameter=tyre_diameter;       
	
	Encoder_precision=EncoderMultiples*Robot_Parament.EncoderAccuracy*Robot_Parament.GearRatio;

  //主动轮周长	
	Wheel_perimeter=Robot_Parament.WheelDiameter*PI;

  //轮距 麦轮车为半轮距  
  Wheel_spacing=Robot_Parament.WheelSpacing; 
	
  //轴距 麦轮车为半轴距	
	Axle_spacing=Robot_Parament.AxleSpacing; 

  //全向轮小车旋转半径	
	Omni_turn_radiaus=Robot_Parament.OmniTurnRadiaus; 
}



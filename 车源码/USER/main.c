#include "system.h"

//任务优先级
#define START_TASK_PRIO	1

//任务堆栈大小	
#define START_STK_SIZE 	256  

//任务句柄
TaskHandle_t StartTask_Handler;

//任务函数
void start_task(void *pvParameters);

//主函数
int main(void)
{ 
  systemInit();//硬件初始化
	
	//创建开始任务
	xTaskCreate((TaskFunction_t )start_task,           //任务函数
							(const char*    )"start_task",          //任务名称
							(uint16_t       )START_STK_SIZE,        //任务堆栈大小
							(void*          )NULL,                  //传递给任务函数的参数
							(UBaseType_t    )START_TASK_PRIO,       //任务优先级
							(TaskHandle_t*  )&StartTask_Handler);    //任务句柄    					
	vTaskStartScheduler();  //开启任务调度	
}
 
//开始任务任务函数
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL(); //进入临界区
	
    //创建任务
	if(Check == 1)
		xTaskCreate(Check_task,  "Check_task",  CHECK_STK_SIZE,  NULL, CHECK_TASK_PRIO,  NULL);
	else if(Check == 0)
		xTaskCreate(Balance_task,  "Balance_task",  BALANCE_STK_SIZE,  NULL, BALANCE_TASK_PRIO,  NULL);	//小车运动控制任务
	if(SysVal.HardWare_Ver==V1_0) 	//IMU数据读取任务,根据不同的硬件版本启动不同的任务.
		xTaskCreate(MPU6050_task,  "IMU_task",  IMU_STK_SIZE,  NULL, IMU_TASK_PRIO,  NULL);
	else if( SysVal.HardWare_Ver==V1_1 )
		xTaskCreate(ICM20948_task,  "IMU_task",  IMU_STK_SIZE,  NULL, IMU_TASK_PRIO,  NULL);
	xTaskCreate(show_task,     "show_task",     SHOW_STK_SIZE,     NULL, SHOW_TASK_PRIO,     NULL); //OLED显示屏显示任务
	xTaskCreate(led_task,      "led_task",      LED_STK_SIZE,      NULL, LED_TASK_PRIO,      NULL);	//LED灯闪烁任务
	xTaskCreate(pstwo_task,    "PSTWO_task",    PS2_STK_SIZE,      NULL, PS2_TASK_PRIO,      NULL);	//读取PS2手柄任务
	xTaskCreate(data_task,     "DATA_task",     DATA_STK_SIZE,     NULL, DATA_TASK_PRIO,     NULL);	//串口3、串口1、CAN发送数据任务
	
    vTaskDelete(StartTask_Handler); //删除开始任务

    taskEXIT_CRITICAL();            //退出临界区
}







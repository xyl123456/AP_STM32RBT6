#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
#include "timer.h"
#include "user_app.h"
#include "exti.h"
#include "wdg.h"

 
/************************************************
 ALIENTEK战舰STM32开发板实验4
 串口实验 
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/


 int main(void)
 {		
	//SystemInit();
	delay_init();	    	 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
  // LED_Init();			     //LED端口初始化
	//KEY_Init();          //初始化与按键连接的硬件接口
	
	//EXTIX_Init();		 	//外部中断初始化
	TIM3_Int_Init(SECOND_10MS_TIME,TIME3_10KHZ_PRE);//10Khz的计数频率，计数到99为10ms 
	myuart_init(1,9600);//初始化串口1,连接PM2.5传感器
	myuart_init(2,9600);//初始化串口2,用于与wifi模块通信，蓝牙模块通信
	 
	 hal_board_init();//初始化板子数据

	IWDG_Init(5,1250);    //与分频数为128,重载值为625,溢出时间为4s	
 	while(1)
	{
		//event_key_handle();
		//event_task_handle();
		uart_receve_handle();
	}	 
 }


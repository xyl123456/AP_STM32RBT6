#include "timer.h"
#include "led.h"
#include "key.h"
#include "wdg.h"


//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//定时器 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/3
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////   	 

//通用定时器3中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器3!
u8 key_fall_flag;

u16 time3_cnt=0;
u16 time3_heart_cnt=0;
u16 time3_led_cnt=0;

u16 key_holdon_ms=0;//按键按下保持时间
u16 key_long_down=0;//长按按键标志
u16 key_short_down=0;//短按按键标志
u16 key_connect_down=0;//连按按键标志
u16 keyupCnt=0;
u16 keyUpFlag=0;//按键抬起标志
u16 doubleclick=0;//连按事件
u16 com_count=0;

u16 heart_count=0;
u16 heart_data_count=0;

u16 RECEVE_HEART=0;//串口接收到心跳数据标志位
u16 sys_heart_flag=0;
u16 sys_send_data_flag=0;

void TIM3_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能
	
	//定时器TIM3初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器


	TIM_Cmd(TIM3, ENABLE);  //使能TIMx					 
}
//函数返回按键按下次数，0表示长按，1,2,3...按键按下次数
void key_count_process(void){
	if(key_fall_flag==1){//有按键按下
		if(KEY0==0)
		{
			if(key_holdon_ms<200){//2s长按按键
				key_holdon_ms++;
			}else{
			//按键按下超过2S,属于长按事件
				key_holdon_ms=0;
				key_long_down=1;
				key_fall_flag=0;//释放按键按下标志位
			}
		}else {
			//按键抬起
			if(key_holdon_ms>5){
				//按键按下间隔大于50ms定时间隔，则默认生成单击事件
				key_holdon_ms=0;
				key_short_down=1;
				key_fall_flag=0;
				
				//如果时间间隔在100---500ms之间，则是连按事件
				if((keyupCnt>10) && (keyupCnt<50)){
					doubleclick=doubleclick+1;
					key_connect_down=1;
					if(doubleclick==2){
					com_count=3;//按键连按次数
					}
				}else {
					key_connect_down=0;
					doubleclick=0;
				}
				
				keyUpFlag=1;//按键抬起，生成按键抬起标志	
			}
			else {
				//按键按下小于50ms，忽略全部,消抖
				key_holdon_ms=0;
				key_long_down=0;
				key_short_down=0;
				key_connect_down=0;
				key_fall_flag=0;
			}
		}
	}
	//单独计数按键是否连按的时间间隔
	if(keyUpFlag){
		keyupCnt++;
	}
	//间隔大于500ms，不属于连按事件
	if(keyupCnt>=50){
		keyupCnt=0;
		keyUpFlag=0;
		key_connect_down=0;
		doubleclick=0;
	}
	
}
//定时器3中断服务程序
void TIM3_IRQHandler(void)   //TIM3中断
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
		{
			time3_cnt++;
			time3_heart_cnt++;
			time3_led_cnt++;
			if(time3_cnt==SEND_SERIAL_DATA){
				time3_cnt=0;
				
				sys_send_data_flag =SEND_SERIAL_EVENT;
			}
			if(time3_heart_cnt==HEART_SERIAL_DATA){
				time3_heart_cnt=0;
				sys_heart_flag=HEART_SERIAL_EVENT;
				
			}
			if(time3_led_cnt==100){
				time3_led_cnt=0;
				//串口标志计数,每秒计数		
			if(RECEVE_HEART==1){
				RECEVE_HEART=0;
				heart_count=0;
			}else{
				heart_count++;
			}
			if(heart_count>180){ 
	 
				//超过180S心跳周期，关闭PM25，关闭电源
					PM25_CON=0;
				PWER_ON_OFF=0;
				heart_count=180;
			}else{
				PM25_CON=1;
			}
				//LED0=!LED0;
			 //IWDG_Feed();//喂狗
			}
			
			//按键长短按事件
			key_count_process();
			
			TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //清除TIMx更新中断标志 
		}
}













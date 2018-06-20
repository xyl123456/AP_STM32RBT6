#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"
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
#define PM25_CON   PBout(1)// PB1
#define WIFI_CONFIG   PBout(9) //pb9

#define WIFI_STATE   GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5) //读取PA5的值
//#define WIFI_STATE   PAin(5)

//#define PWER_ON_OFF    PBout(9)// PB9

//#define POW_CHARGE  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)//读取PB12的值


extern u16 sys_heart_flag;
extern u16 sys_send_data_flag;
extern u16 sys_wifi_state_flag;//wifi状态标志位

extern void TIM3_Int_Init(u16 arr,u16 psc);
extern u8 key_fall_flag;


extern u16 key_long_down;//长按按键标志
extern u16 key_short_down;//短按按键标志
extern u16 key_connect_down;//连按按键标志
extern u16 doubleclick;//连按事件次数
extern u16 com_count;
extern u16 power_charge_temp;//充电状态标志


extern u16 RECEVE_HEART;
void key_count_process(void);
void power_charge(void);
#endif

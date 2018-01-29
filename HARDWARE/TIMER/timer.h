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
#define PWER_ON_OFF    PBout(9)// PB9
//#define POWER_CON  PBout(12)

extern u16 sys_heart_flag;
extern u16 sys_send_data_flag;

extern void TIM3_Int_Init(u16 arr,u16 psc);
extern u8 key_fall_flag;


extern u16 key_long_down;//长按按键标志
extern u16 key_short_down;//短按按键标志
extern u16 key_connect_down;//连按按键标志
extern u16 doubleclick;//连按事件次数
extern u16 com_count;

extern u16 RECEVE_HEART;
void key_count_process(void);
#endif

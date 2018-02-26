#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//��ʱ�� ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/3
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////   
#define PM25_CON   PBout(1)// PB1 
#define PWER_ON_OFF    PBout(9)// PB9

#define POW_CHARGE  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)//��ȡPB12��ֵ


extern u16 sys_heart_flag;
extern u16 sys_send_data_flag;

extern void TIM3_Int_Init(u16 arr,u16 psc);
extern u8 key_fall_flag;


extern u16 key_long_down;//����������־
extern u16 key_short_down;//�̰�������־
extern u16 key_connect_down;//����������־
extern u16 doubleclick;//�����¼�����
extern u16 com_count;
extern u16 power_charge_temp;//���״̬��־


extern u16 RECEVE_HEART;
void key_count_process(void);
void power_charge(void);
#endif

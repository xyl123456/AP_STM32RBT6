#include "timer.h"
#include "led.h"
#include "key.h"
#include "wdg.h"


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

//ͨ�ö�ʱ��3�жϳ�ʼ��
//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//����ʹ�õ��Ƕ�ʱ��3!
u8 key_fall_flag;

u16 time3_cnt=0;
u16 time3_heart_cnt=0;
u16 time3_led_cnt=0;

u16 key_holdon_ms=0;//�������±���ʱ��
u16 key_long_down=0;//����������־
u16 key_short_down=0;//�̰�������־
u16 key_connect_down=0;//����������־
u16 keyupCnt=0;
u16 keyUpFlag=0;//����̧���־
u16 doubleclick=0;//�����¼�
u16 com_count=0;

u16 heart_count=0;
u16 heart_data_count=0;

u16 RECEVE_HEART=0;//���ڽ��յ��������ݱ�־λ
u16 sys_heart_flag=0;
u16 sys_send_data_flag=0;

void TIM3_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��
	
	//��ʱ��TIM3��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���


	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx					 
}
//�������ذ������´�����0��ʾ������1,2,3...�������´���
void key_count_process(void){
	if(key_fall_flag==1){//�а�������
		if(KEY0==0)
		{
			if(key_holdon_ms<200){//2s��������
				key_holdon_ms++;
			}else{
			//�������³���2S,���ڳ����¼�
				key_holdon_ms=0;
				key_long_down=1;
				key_fall_flag=0;//�ͷŰ������±�־λ
			}
		}else {
			//����̧��
			if(key_holdon_ms>5){
				//�������¼������50ms��ʱ�������Ĭ�����ɵ����¼�
				key_holdon_ms=0;
				key_short_down=1;
				key_fall_flag=0;
				
				//���ʱ������100---500ms֮�䣬���������¼�
				if((keyupCnt>10) && (keyupCnt<50)){
					doubleclick=doubleclick+1;
					key_connect_down=1;
					if(doubleclick==2){
					com_count=3;//������������
					}
				}else {
					key_connect_down=0;
					doubleclick=0;
				}
				
				keyUpFlag=1;//����̧�����ɰ���̧���־	
			}
			else {
				//��������С��50ms������ȫ��,����
				key_holdon_ms=0;
				key_long_down=0;
				key_short_down=0;
				key_connect_down=0;
				key_fall_flag=0;
			}
		}
	}
	//�������������Ƿ�������ʱ����
	if(keyUpFlag){
		keyupCnt++;
	}
	//�������500ms�������������¼�
	if(keyupCnt>=50){
		keyupCnt=0;
		keyUpFlag=0;
		key_connect_down=0;
		doubleclick=0;
	}
	
}
//��ʱ��3�жϷ������
void TIM3_IRQHandler(void)   //TIM3�ж�
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //���TIM3�����жϷ������
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
				//���ڱ�־����,ÿ�����		
			if(RECEVE_HEART==1){
				RECEVE_HEART=0;
				heart_count=0;
			}else{
				heart_count++;
			}
			if(heart_count>180){ 
	 
				//����180S�������ڣ��ر�PM25���رյ�Դ
					PM25_CON=0;
				PWER_ON_OFF=0;
				heart_count=180;
			}else{
				PM25_CON=1;
			}
				//LED0=!LED0;
			 //IWDG_Feed();//ι��
			}
			
			//�������̰��¼�
			key_count_process();
			
			TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //���TIMx�����жϱ�־ 
		}
}













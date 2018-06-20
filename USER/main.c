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
 ALIENTEKս��STM32������ʵ��4
 ����ʵ�� 
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/


 int main(void)
 {		
	//SystemInit();
	delay_init();	    	 //��ʱ������ʼ��	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
  // LED_Init();			     //LED�˿ڳ�ʼ��
	//KEY_Init();          //��ʼ���밴�����ӵ�Ӳ���ӿ�
	
	//EXTIX_Init();		 	//�ⲿ�жϳ�ʼ��
	TIM3_Int_Init(SECOND_10MS_TIME,TIME3_10KHZ_PRE);//10Khz�ļ���Ƶ�ʣ�������99Ϊ10ms 
	myuart_init(1,9600);//��ʼ������1,����PM2.5������
	myuart_init(2,9600);//��ʼ������2,������wifiģ��ͨ�ţ�����ģ��ͨ��
	 
	 hal_board_init();//��ʼ����������

	IWDG_Init(5,1250);    //���Ƶ��Ϊ128,����ֵΪ625,���ʱ��Ϊ4s	
 	while(1)
	{
		//event_key_handle();
		//event_task_handle();
		uart_receve_handle();
	}	 
 }


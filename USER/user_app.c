#include "user_app.h"
#include "usart.h"
#include "timer.h"
#include "malloc.h"	
#include "led.h"
#include "delay.h"
#include "key.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stmflash.h"
#include "malloc.h"	
#include "sht3x.h"
#include "adc.h"
#include "timer.h"
 

//ע����Ϣ����
unsigned char HEARD_DATA[2]={0xEB,0x90};
unsigned char HEARD_LENGTH[2]={0x00,0x14};
u16 DEV_MAC_ADDR[4]={0x00,0x00,0x00,0x00};
unsigned char VERSION[3]={0x00,0x00,0x01};
unsigned char HARD_VERSION[3]={0x00,0x00,0x01};
unsigned char HEART_TIME[3]={0x00,0x00,0x1E};
unsigned char TAIL_DATA[2]={0x0D,0x0A};

//wifiҪ���·�ע������
u8 WIFI_CMD[13]={0xEB,0x90,0x00,0x0B,0x0F,0x00,0x00,0x00,0x00,0x00,0x1A,0x0D,0x0A};
//��������
unsigned char HEART_LENGTH[2]={0x00,0x0B};
//�����ϱ�
unsigned char UPDATA_LENGTH[2]={0x00,0x1D};
unsigned char PM25_DATA[3]={0x01,0x00,0x28};
unsigned char PM03_DATA[3]={0x0E,0x00,0x28};
unsigned char TEM_DATA[3];
unsigned char HUM_DATA[3];
unsigned char datatemp[4];
//���������ϱ���PM2.5���������ϱ�����
Data_up_t data_up_t;
//����PM2.5���ݽṹ��
PM25_up_t pm25_up_t;
//��ʪ�Ƚṹ��
SHT30_DATA_STRUCT   SHT30_data;

uint16_t  Sum_data(unsigned char buf[],int start,int stop){
	uint16_t sum_data=0;
	int i;
	for(i=start;i<=stop;i++){
		sum_data=sum_data+buf[i];
	}
	return sum_data;
}
//ע�������ϱ�
void send_rejest_data(void){
	uint16_t check_data;
	unsigned char check_buf[2];
	Rejest_up_t rejest_up_t;

	mymemcpy(rejest_up_t.data_core.Head_byte,(void *)HEARD_DATA,2);
	mymemcpy(rejest_up_t.data_core.Data_length,(void *)HEARD_LENGTH,2);
	rejest_up_t.data_core.Data_type=0x02;
	mymemcpy(rejest_up_t.data_core.MAC_addr,DEV_MAC_ADDR,4);
	
	mymemcpy(rejest_up_t.data_core.Version,(void *)VERSION,3);
	
	mymemcpy(rejest_up_t.data_core.HardVersion,(void *)HARD_VERSION,3);
	
	mymemcpy(rejest_up_t.data_core.Heart_time,(void *)HEART_TIME,3);

	check_data=Sum_data(rejest_up_t.data_buf,2,HAL_REJEST_LENGTH-5);
	check_buf[0]=check_data>>8;
  check_buf[1]=check_data;
	
	mymemcpy(rejest_up_t.data_core.Check_code,check_buf,2);
	mymemcpy(rejest_up_t.data_core.Tial,(void *)TAIL_DATA,2);
   
	myuart_send(HAL_UART2,rejest_up_t.data_buf,HAL_REJEST_LENGTH);
	
  mymemset(rejest_up_t.data_buf,0,HAL_REJEST_LENGTH);
}

void 	GPIO_output_Init(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
 	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);	 //ʹ��PB�˿�ʱ��
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_5;//PA5��Ϊ���룬����Ƿ�wifi�������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //���óɸ�������
 	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIO_PA5
	
	//GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_12;//PB12��Ϊ����Ƿ��г����źŶ˿�
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //���ó���������
 	//GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO_PB12
	
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;				 //pm25-->PB.1  PB12 �˿�����
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;		 //IO���ٶ�Ϊ2MHz
// GPIO_Init(GPIOB, &GPIO_InitStructure);	                                    				 //�����趨������ʼ��GPIOB.12
// GPIO_ResetBits(GPIOB,GPIO_Pin_12);				 //PB12 ���0
	
 //����PM25�Ŀ����͹ر�
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;				 //pm25-->PB.1  PB1 �˿�����
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;		 //IO���ٶ�Ϊ2MHz
 GPIO_Init(GPIOB, &GPIO_InitStructure);	                                    				 //�����趨������ʼ��GPIOB.1
 GPIO_SetBits(GPIOB,GPIO_Pin_1);				 //PB.1���1,Ĭ���ϵ��PM2.5
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;				 //PB.9 �˿�����,�������wifiģ�������ģʽ
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;		 //IO���ٶ�Ϊ2MHz
 GPIO_Init(GPIOB, &GPIO_InitStructure);	                                    				 //�����趨������ʼ��GPIOB.9
 GPIO_SetBits(GPIOB,GPIO_Pin_9);				 //PB.9 ���1���͵�ƽ��ʾ��������ģʽ
	
}
void hal_board_init(void) {
	GPIO_output_Init();
	
	USART_GetFlagStatus(USART1, USART_FLAG_TC);
	USART_GetFlagStatus(USART2, USART_FLAG_TC);
	//��ȡflash���豸��ַ 4 byte
	STMFLASH_Read(FLASH_SAVE_ADDR,(u16*)DEV_MAC_ADDR,4);
	
	
	//Adc_Init();		  		//ADC��ʼ��
	//STMFLASH_Write(FLASH_SAVE_ADDR,(u16*)DEV_MAC_ADDR,4);
	SHT3X_Init();           //��ʼ��I2C��ʪ��ģ��
	//send_rejest_data();
	//PWER_ON_OFF=1;//���ڿ��ƿ�����·��IO��
}
//���������ϱ�
void send_heart_data(void){
	uint16_t check_data;
	unsigned char check_buf[2];
	Heart_up_t heart_up_t;
	mymemcpy(heart_up_t.data_core.Head_byte,(void *)HEARD_DATA,2);
	mymemcpy(heart_up_t.data_core.Data_length,(void *)HEART_LENGTH,2);
	heart_up_t.data_core.Data_type=0x01;
	mymemcpy(heart_up_t.data_core.MAC_addr,DEV_MAC_ADDR,4);
	
	check_data=Sum_data(heart_up_t.data_buf,2,HAL_HEART_LENGTH-5);
	check_buf[0]=check_data>>8;
  check_buf[1]=check_data;
	mymemcpy(heart_up_t.data_core.Check_code,check_buf,2);
	mymemcpy(heart_up_t.data_core.Tial,(void *)TAIL_DATA,2);
	
	myuart_send(HAL_UART2,heart_up_t.data_buf,HAL_HEART_LENGTH);
	
}
void hal_wakeup_lowpower(void){
	
//	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
}

void hal_set_lowpower(void){
//	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, DISABLE);
//	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, DISABLE);
 
}

void send_serial_data(void) {
	uint16_t check_data;
	unsigned char check_buf[2];
	
	u8 temp;
	int temp_buffer;

	mymemcpy(data_up_t.data_core.Head_byte,(void *)HEARD_DATA,2);
	mymemcpy(data_up_t.data_core.Data_length,(void *)UPDATA_LENGTH,2);
	data_up_t.data_core.Data_type=0x04;
	mymemcpy(data_up_t.data_core.MAC_addr,DEV_MAC_ADDR,4);
	mymemcpy(data_up_t.data_core.PM25,PM25_DATA,3);
	mymemcpy(data_up_t.data_core.PM03,PM03_DATA,3);
	
	temp = SHT3X_GetTempAndHumi(&SHT30_data.Temp,&SHT30_data.Humi, REPEATAB_HIGH, MODE_POLLING, 50);

if(temp==NO_ERROR){
	
			temp_buffer=(int)SHT30_data.Temp+0x5E;//ԭ����100������оƬ�Ͱ��ӵ��²����94
			SHT30_data.Temp_byte[0]=0x03;
			SHT30_data.Temp_byte[1]=(temp_buffer>>8)&0xff;
			SHT30_data.Temp_byte[2]=temp_buffer&0xff;
			
			SHT30_data.Humi_byte[0]=0x04;
			SHT30_data.Humi_byte[1]=((int)SHT30_data.Humi>>8)&0xff;
			SHT30_data.Humi_byte[2]= (int)SHT30_data.Humi&0xff;
		
			mymemcpy(TEM_DATA,SHT30_data.Temp_byte,3);
			mymemcpy(HUM_DATA,SHT30_data.Humi_byte,3);
}
	
		mymemcpy(data_up_t.data_core.TEM,TEM_DATA,3);
		mymemcpy(data_up_t.data_core.HUM,HUM_DATA,3);

	check_data=Sum_data(data_up_t.data_buf,2,HAL_DATAUP_LENGTH-5);
	check_buf[0]=check_data>>8;
  check_buf[1]=check_data;
	
	mymemcpy(data_up_t.data_core.Check_code,check_buf,2);
	mymemcpy(data_up_t.data_core.Tial,(void *)TAIL_DATA,2);
	
	myuart_send(HAL_UART2,data_up_t.data_buf,HAL_DATAUP_LENGTH);
	
	//STMFLASH_Read(FLASH_SAVE_ADDR,(u16*)datatemp,4);
	
	//myuart_send(HAL_UART2,datatemp,4);

	
}

void event_task_handle(void) {
	
		if(sys_wifi_state_flag==1){
			//��wifi������AP��Żᷢ������
		send_serial_data();	
		}

}
void App_process_pm25(u8 buf[],u8 len)
{
	
	uint16_t pm03_data;
	uint16_t pm05_data;	
	uint16_t pm03_real_data;
	unsigned char pm03_buf[2];
	pm03_data=(((uint16_t)pm25_up_t.data_core.pm03_kq[0])<<8) + pm25_up_t.data_core.pm03_kq[1];
	pm05_data=(((uint16_t)pm25_up_t.data_core.pm05_kq[0])<<8) + pm25_up_t.data_core.pm05_kq[1];
	pm03_real_data=(pm03_data-pm05_data)/10;
	pm03_buf[0]=pm03_real_data>>8;
	pm03_buf[1]=pm03_real_data;
	mymemcpy(pm25_up_t.data_buf,buf,len);
	mymemcpy(PM25_DATA+1,pm25_up_t.data_core.pm25_dq,2);
	mymemcpy(PM03_DATA+1,pm03_buf,2);	
	
}

void App_process_uart(u8 port,u8 buf[],u8 len){
	uint16_t check_data;
	uint16_t up_check_data;
	int i;
	unsigned char check_buf[2];
	unsigned char up_check_buf[2];//�����У������
	
	unsigned char commend_up_data[64];//�����ϴ�����
	unsigned char commend_buf[24];//�·���������,���֧��8������ͬʱ�·�
	
	check_data=Sum_data(buf,2,len-5);
	check_buf[0]=check_data>>8;
	check_buf[1]=check_data;
	
	if(mymemcmp(buf+(len-4),check_buf,2)==0){
		
	if(port==HAL_UART2){
		//����1����ͨ�ţ������·��Ŀ�������
	  if(buf[4]==0x05){
			//��������
			mymemcpy(commend_buf,buf+17,len-21);
			//myuart_send(HAL_UART1,commend_buf,len-21);
			for(i=0;i<(len-21)/3;i=i+3){
				switch(commend_buf[i]){
					case 0x81:
						if(commend_buf[i+2]==0x0A){
							//����
						PM25_CON=1;
						}else if(commend_buf[i+2]==0x0B){
							//�ػ�
						PM25_CON=0;
						}
						break;
					default:
						break;		
				}
			}
			//���ؿ�������
			up_check_data=check_data+1;
			up_check_buf[0]=up_check_data>>8;
			up_check_buf[1]=up_check_data;

			mymemcpy(commend_up_data,buf,len);
			commend_up_data[4]=0x06;
			mymemcpy(commend_up_data+(len-4),up_check_buf,2);
			myuart_send(HAL_UART2,commend_up_data,len);
			
			mymemset(commend_buf,0,24);
			mymemset(commend_up_data,0,len);
			
		}
	}else if(port==HAL_UART1){
		//������1���յ�����:�����ʽEB 90 
		
		}
}else {
	len =0;
	mymemset(buf,0,sizeof((u8*)buf));
	}	
}

//���ڴ���ص�����
void uart_receve_handle(void)
{
	if(USART_RX_STA&0x8000)
		{	
			u8 uart_len;
			u8 uart_commandbuf[USART_REC_LEN];
			uart_len=USART_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
			mymemcpy(uart_commandbuf,USART_RX_BUF,uart_len);
			//myuart_send(HAL_UART2,uart_commandbuf,uart_len);
			App_process_pm25(uart_commandbuf,uart_len);
			USART_RX_STA=0;
			mymemset(USART_RX_BUF,0,USART_REC_LEN);
			mymemset(uart_commandbuf,0,USART_REC_LEN);
		}		
	else if(USART2_RX_STA&0x8000)
		{
			u8 uart2_len;
			u8 uart2_commandbuf[USART_REC_LEN];
			uart2_len=USART2_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
			mymemcpy(uart2_commandbuf,USART2_RX_BUF,uart2_len);
			
			if(mymemcmp(uart2_commandbuf+5,DEV_MAC_ADDR,4)==0){
				
				if(mymemcmp(uart2_commandbuf,(void *)HEARD_DATA,2)==0){
						App_process_uart(HAL_UART2,uart2_commandbuf,uart2_len);
					}
				}
			if(mymemcmp(uart2_commandbuf,(void *)WIFI_CMD,13)==0)
			{
				send_rejest_data();//wifi����ע����Ϣ
			}
			
			if((uart2_len==HAL_MODIFY_LENGTH)&(uart2_commandbuf[4]==0xFD)){
					//�޸��豸ID
					Dev_dp_t dev_dp_t;//���յ�����
					Dev_up_t dev_up_t;//���ڷ����ϱ�������

					uint16_t check_data;
					unsigned char check_buf[2];
					mymemcpy(dev_dp_t.data_buf,uart2_commandbuf,uart2_len);
	
					mymemcpy(dev_up_t.data_buf,uart2_commandbuf,uart2_len);	
			
					mymemcpy(DEV_MAC_ADDR,dev_dp_t.data_core.MAC_addr,4);	
					STMFLASH_Write(FLASH_SAVE_ADDR,(u16*)DEV_MAC_ADDR,4);	
					//�������õ�ַ����
					dev_up_t.data_core.Data_type=0xFE;
					check_data=Sum_data(dev_up_t.data_buf,2,HAL_MODIFY_LENGTH-5);
					check_buf[0]=check_data>>8;
					check_buf[1]=check_data;
					mymemcpy(dev_up_t.data_core.Check_code,check_buf,2);
			
					myuart_send(HAL_UART2,dev_up_t.data_buf,HAL_MODIFY_LENGTH);
				}
		
			USART2_RX_STA=0;
			uart2_len=0;
			mymemset(USART2_RX_BUF,0,USART_REC_LEN);
			mymemset(uart2_commandbuf,0,USART_REC_LEN);
			
		}
	
}


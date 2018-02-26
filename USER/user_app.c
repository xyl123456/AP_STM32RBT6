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
 


//ע����Ϣ����
unsigned char HEARD_DATA[2]={0xEB,0x90};
unsigned char HEARD_LENGTH[2]={0x00,0x14};
unsigned char DEV_MAC_ADDR[4]={0x00,0x00,0x00,0x00};
unsigned char VERSION[3]={0x20,0x00,0x00};
unsigned char HARD_VERSION[3]={0x10,0x00,0x00};
unsigned char HEART_TIME[3]={0x00,0x00,0x1E};
unsigned char TAIL_DATA[2]={0x0D,0x0A};

//��������
unsigned char HEART_LENGTH[2]={0x00,0x0B};
//�����ϱ�
unsigned char UPDATA_LENGTH[2]={0x00,0x1D};
unsigned char PM25_DATA[3]={0x01,0x00,0x2F};
unsigned char PM03_DATA[3]={0x0E,0x00,0x14};
unsigned char TEM_DATA[3];
unsigned char HUM_DATA[3];
unsigned char POW_DATA[3];//������ֵ,0X0A,0X00,0X64
unsigned char POW_sta[3];//��س��״̬ ��0x0F ,0X00,0X01

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
	mymemcpy(rejest_up_t.data_core.Head_byte,HEARD_DATA,2);
	mymemcpy(rejest_up_t.data_core.Data_length,HEARD_LENGTH,2);
	rejest_up_t.data_core.Data_type=0x02;
	mymemcpy(rejest_up_t.data_core.MAC_addr,DEV_MAC_ADDR,4);
	mymemcpy(rejest_up_t.data_core.Version,VERSION,3);
	mymemcpy(rejest_up_t.data_core.HardVersion,HARD_VERSION,3);
	mymemcpy(rejest_up_t.data_core.Heart_time,HEART_TIME,3);

	check_data=Sum_data(rejest_up_t.data_buf,2,HAL_REJEST_LENGTH-5);
	check_buf[0]=check_data>>8;
  check_buf[1]=check_data;
	
	mymemcpy(rejest_up_t.data_core.Check_code,check_buf,2);
	mymemcpy(rejest_up_t.data_core.Tial,TAIL_DATA,2);
   
	myuart_send(HAL_UART2,rejest_up_t.data_buf,HAL_REJEST_LENGTH);
}

void 	GPIO_output_Init(void){
	//����˿�����PB1----pm2.5,PB12-----��˵�Դ
	GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //ʹ��PB�˿�ʱ��
	
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_12;//PB12��Ϊ����Ƿ��г����źŶ˿�
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //���ó���������
 	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO_PB12
	
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;				 //pm25-->PB.1  PB12 �˿�����
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;		 //IO���ٶ�Ϊ2MHz
// GPIO_Init(GPIOB, &GPIO_InitStructure);	                                    				 //�����趨������ʼ��GPIOB.1
// GPIO_ResetBits(GPIOB,GPIO_Pin_12);				 //PB12 ���0
	
 //����PM25�Ŀ����͹ر�
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;				 //pm25-->PB.1  PB1 �˿�����
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;		 //IO���ٶ�Ϊ2MHz
 GPIO_Init(GPIOB, &GPIO_InitStructure);	                                    				 //�����趨������ʼ��GPIOB.1
 GPIO_ResetBits(GPIOB,GPIO_Pin_1);				 //PB.1���0
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;				 //LED0-->PB.9 �˿�����,������Ƶ�Դ���ػ�
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;		 //IO���ٶ�Ϊ2MHz
 GPIO_Init(GPIOB, &GPIO_InitStructure);	                                    				 //�����趨������ʼ��GPIOB.1
 GPIO_ResetBits(GPIOB,GPIO_Pin_9);				 //PB.9 ���0
	
}
void hal_board_init(void) {
	
	USART_GetFlagStatus(USART1, USART_FLAG_TC);
	USART_GetFlagStatus(USART2, USART_FLAG_TC);
	//��ȡflash���豸��ַ 4 byte
	STMFLASH_Read(FLASH_SAVE_ADDR,(u16*)DEV_MAC_ADDR,4);
	
	GPIO_output_Init();
	Adc_Init();		  		//ADC��ʼ��
	//STMFLASH_Write(FLASH_SAVE_ADDR,(u16*)DEV_MAC_ADDR,4);
	SHT3X_Init();           //��ʼ��I2C��ʪ��ģ��
	//send_rejest_data();
	PWER_ON_OFF=1;
	PM25_CON=1;
}
//���������ϱ�
void send_heart_data(void){
	uint16_t check_data;
	unsigned char check_buf[2];
	Heart_up_t heart_up_t;
	mymemcpy(heart_up_t.data_core.Head_byte,HEARD_DATA,2);
	mymemcpy(heart_up_t.data_core.Data_length,HEART_LENGTH,2);
	heart_up_t.data_core.Data_type=0x01;
	mymemcpy(heart_up_t.data_core.MAC_addr,DEV_MAC_ADDR,4);
	
	check_data=Sum_data(heart_up_t.data_buf,2,HAL_HEART_LENGTH-5);
	check_buf[0]=check_data>>8;
  check_buf[1]=check_data;
	mymemcpy(heart_up_t.data_core.Check_code,check_buf,2);
	mymemcpy(heart_up_t.data_core.Tial,TAIL_DATA,2);
	
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
	//ADC�ɼ�
	u16 adc_data;
	//float adc_temp;
	u8 temp;
	int temp_buffer;

	mymemcpy(data_up_t.data_core.Head_byte,HEARD_DATA,2);
	mymemcpy(data_up_t.data_core.Data_length,UPDATA_LENGTH,2);
	data_up_t.data_core.Data_type=0x04;
	mymemcpy(data_up_t.data_core.MAC_addr,DEV_MAC_ADDR,4);
	mymemcpy(data_up_t.data_core.PM25,PM25_DATA,3);
	mymemcpy(data_up_t.data_core.PM03,PM03_DATA,3);
	
	temp = SHT3X_GetTempAndHumi(&SHT30_data.Temp,&SHT30_data.Humi, REPEATAB_HIGH, MODE_POLLING, 50);

if(temp==NO_ERROR){
	
			temp_buffer=(int)SHT30_data.Temp+0x03E8;
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

	//��ȡADCֵ
		adc_data=Get_Adc_Average(ADC_Channel_5,10);
		//adc_temp=(float)adc_data*(3.3/4096);
		//adc_data=adc_temp;
   POW_DATA[0]=0x0A;
	 POW_DATA[1]=0x00;
	if(adc_data>=0x061F){
		POW_DATA[2]=0x64;
	}
	if((adc_data<0x061F)&(adc_data>=0x05D1)){
		POW_DATA[2]=0x50;
	}
	if((adc_data<0x05D1)&(adc_data>=0x056E)){
		POW_DATA[2]=0x3C;
	}
	if((adc_data<0x056E)&(adc_data>=0x0510)){
		POW_DATA[2]=0x28;
	}
		if(adc_data<0x0510){
		POW_DATA[2]=0x0A;
	}

  // POW_DATA[1]=(adc_data>>8)&0xFF;
	 //POW_DATA[2]=adc_data&0xFF;
	mymemcpy(data_up_t.data_core.POW,POW_DATA,3);
	POW_sta[0]=0x0F;
	POW_sta[1]=0x00;
	if(power_charge_temp==1){
	POW_sta[2]=0x01;//���״̬
	}else{
		POW_sta[2]=0x00;
	}
	mymemcpy(data_up_t.data_core.POW_STA,POW_sta,3);

	check_data=Sum_data(data_up_t.data_buf,2,HAL_DATAUP_LENGTH-5);
	check_buf[0]=check_data>>8;
  check_buf[1]=check_data;
	
	mymemcpy(data_up_t.data_core.Check_code,check_buf,2);
	mymemcpy(data_up_t.data_core.Tial,TAIL_DATA,2);
	
	myuart_send(HAL_UART2,data_up_t.data_buf,HAL_DATAUP_LENGTH);
	
	//STMFLASH_Read(FLASH_SAVE_ADDR,(u16*)datatemp,4);
	
	//myuart_send(HAL_UART2,datatemp,4);
	
}

void event_task_handle(void) {
	

	if(sys_send_data_flag==SEND_SERIAL_EVENT){
		//���������ϱ���Ϣ
		send_serial_data();
		sys_send_data_flag =0;
	}
//	if(sys_heart_flag==HEART_SERIAL_EVENT){
//		//�����¼�
//		send_heart_data();
//		sys_heart_flag=0;
//	}
}
void App_process_pm25(u8 buf[],u8 len)
{
	mymemcpy(pm25_up_t.data_buf,buf,len);
	mymemcpy(PM25_DATA+1,pm25_up_t.data_core.pm25_dq,2);
	mymemcpy(PM03_DATA+1,pm25_up_t.data_core.pm10_dq,2);	
}

void App_process_uart(u8 port,u8 buf[],u8 len){
	uint16_t check_data;
	uint16_t up_check_data;
	int i;
	unsigned char check_buf[2];
	unsigned char up_check_buf[2];//�����У������
	
	unsigned char commend_up_data[64];//�����ϴ�����
	unsigned char commend_buf[24];//�·���������
	
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

						}else if(commend_buf[i+2]==0x0B){
							//�ػ�

						}
						break;
					case 0x82:
						if(commend_buf[i+2]==0x0C){
					
							//�����ӿ�
						}else if(commend_buf[i+2]==0x0D){
							//�����ӹ�
								
						}else if(commend_buf[i+2]==0x0E){
							//����ƿ�
								
						}else if(commend_buf[i+2]==0x0F){
							//����ƹ�
							
						}
						break;
					case 0x84:
						if(commend_buf[i+2]==0x0B){
							//����
						}else if(commend_buf[i+2]==0x10){
							//1����
								
						}else if(commend_buf[i+2]==0x11){
							//2����
								
						}else if(commend_buf[i+2]==0x12){
							//3����
								
						}else if(commend_buf[i+2]==0x13){
							//4����
								
						}
						break;
					case 0x85:
							if(commend_buf[i+2]==0x1A){
							//������ʱ����
						}else if(commend_buf[i+2]==0x1B){
							//�رն�ʱ����
						}else if(commend_buf[i+2]==0x47){
							//��ʱ1Сʱ�Զ��ػ�
						}else if(commend_buf[i+2]==0x48){
							//��ʱ2Сʱ�Զ��ػ�
						}else if(commend_buf[i+2]==0x49){
							//��ʱ4Сʱ�Զ��ػ�
						}else if(commend_buf[i+2]==0x4A){
							//��ʱ8Сʱ�Զ��ػ�
						}
						break;
					case 0x88:
							if(commend_buf[i+2]==0x1C){
							//��ʱһ����Ч
						}else if(commend_buf[i+2]==0x1D){
							//��ʱÿ����Ч
						}else if(commend_buf[i+2]==0x1E){
							//��ʱ��һ��Ч
						}else if(commend_buf[i+2]==0x1F){
							//��ʱ�ܶ���Ч
						}else if(commend_buf[i+2]==0x20){
							//��ʱ������Ч
						}else if(commend_buf[i+2]==0x21){
							//��ʱ������Ч
						}else if(commend_buf[i+2]==0x22){
							//��ʱ������Ч
						}else if(commend_buf[i+2]==0x23){
							//��ʱ������Ч
						}else if(commend_buf[i+2]==0x24){
							//��ʱ������Ч
						}
						break;
					case 0x92:
						if(commend_buf[i+2]==0x2C){
							//����/��ͯ����
							
						}else if(commend_buf[i+2]==0x2D){
							//����/��ͯ����
								
						}
						break;
					case 0x91:
						if(commend_buf[i+2]==0x3A){
							//�����豸���ر�����ģʽ
							
						}else if(commend_buf[i+2]==0x3B){
							//����˯��ģʽ
								
						}else if(commend_buf[i+2]==0x3C){
							//�����ֶ�ģʽ
								
						}
						break;
					case 0x98:
						if(commend_buf[i+2]==0x40){
							//��Ч������ʼ������������
							
						}else if(commend_buf[i+2]==0x41){
							//��Ч������ʼ������������
						}else if(commend_buf[i+2]==0x42){
							//��Ч������ʼ������������
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

void hold_power_handle(void){
	if(KEY0==0){
		PWER_ON_OFF=1;
	}
}
//����������
void event_key_handle(void){
	if(key_long_down==1){
		//�����¼�,�ػ�
		PM25_CON=0;
		PWER_ON_OFF=0;
		RECEVE_HEART=0;
		key_long_down=0;
		//���õ͹���ģʽ
   // hal_set_lowpower();
		
	}
	if(key_short_down==1){
		//�̰������¼�
		PWER_ON_OFF=1;
		PM25_CON=1;
		RECEVE_HEART=1;
		key_short_down=0;
		//hal_wakeup_lowpower();
	}
	if(key_connect_down==1){
		//��������
		if(com_count==3){
		}
		key_connect_down=0;
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
			
//			if(mymemcmp(uart2_commandbuf+5,DEV_MAC_ADDR,4)==0){
//				
//			App_process_uart(HAL_UART2,uart2_commandbuf,uart2_len);
//			
//			}
			
				if((uart2_len==HAL_HEART_LENGTH)&(uart2_commandbuf[4]==0x01)){
					//�յ�������������������,��������
					RECEVE_HEART=1;
					send_serial_data();
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
			
			//myuart_send(HAL_UART1,uart_commandbuf,uart_len);
			
			USART2_RX_STA=0;
			uart2_len=0;
			mymemset(USART2_RX_BUF,0,USART_REC_LEN);
			mymemset(uart2_commandbuf,0,USART_REC_LEN);
			
		}
	
}

//���ڴ���ص�����
//void uart_receve_handle(void)
//{
//	if(USART_RX_STA&0x8000)
//		{	
//			u8 uart_len;
//			u8 uart_commandbuf[USART_REC_LEN];
//			uart_len=USART_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
//			mymemcpy(uart_commandbuf,USART_RX_BUF,uart_len);
//			
//			if(mymemcmp(uart_commandbuf+5,DEV_MAC_ADDR,4)==0){
//				
//			App_process_uart(HAL_UART1,uart_commandbuf,uart_len);
//			
//			}
//			if((uart_len==HAL_MODIFY_LENGTH)&(uart_commandbuf[4]==0xFD)){
//					//�޸��豸ID
//					Dev_dp_t dev_dp_t;//���յ�����
//					Dev_up_t dev_up_t;//���ڷ����ϱ�������
//					uint16_t check_data;
//					unsigned char check_buf[2];
//					mymemcpy(dev_dp_t.data_buf,uart_commandbuf,uart_len);
//					mymemcpy(dev_up_t.data_buf,uart_commandbuf,uart_len);	
//			
//					mymemcpy(DEV_MAC_ADDR,dev_dp_t.data_core.MAC_addr,4);	
//					STMFLASH_Write(FLASH_SAVE_ADDR,(u16*)DEV_MAC_ADDR,4);	
//					//�������õ�ַ����
//					dev_up_t.data_core.Data_type=0xFE;
//					check_data=Sum_data(dev_up_t.data_buf,2,HAL_MODIFY_LENGTH-5);
//					check_buf[0]=check_data>>8;
//					check_buf[1]=check_data;
//					mymemcpy(dev_up_t.data_core.Check_code,check_buf,2);
//			
//					myuart_send(HAL_UART1,dev_up_t.data_buf,HAL_MODIFY_LENGTH);
//				}
//			
//			//myuart_send(HAL_UART1,uart_commandbuf,uart_len);
//			
//			USART_RX_STA=0;
//			uart_len=0;
//			mymemset(USART_RX_BUF,0,USART_REC_LEN);
//			mymemset(uart_commandbuf,0,USART_REC_LEN);
//			
//		}
//		if(USART2_RX_STA&0x8000)
//		{	
//			u8 uart2_len;
//			u8 uart2_commandbuf[USART_REC_LEN];
//			uart2_len=USART2_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
//			mymemcpy(uart2_commandbuf,USART2_RX_BUF,uart2_len);
//			myuart_send(HAL_UART2,uart2_commandbuf,uart2_len);
//			USART2_RX_STA=0;
//			mymemset(USART2_RX_BUF,0,USART_REC_LEN);
//			mymemset(uart2_commandbuf,0,USART_REC_LEN);
//		}		
//}


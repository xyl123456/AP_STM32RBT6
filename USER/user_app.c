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
 


//注册信息数据
unsigned char HEARD_DATA[2]={0xEB,0x90};
unsigned char HEARD_LENGTH[2]={0x00,0x14};
unsigned char DEV_MAC_ADDR[4]={0x00,0x00,0x00,0x00};
unsigned char VERSION[3]={0x20,0x00,0x00};
unsigned char HARD_VERSION[3]={0x10,0x00,0x00};
unsigned char HEART_TIME[3]={0x00,0x00,0x1E};
unsigned char TAIL_DATA[2]={0x0D,0x0A};

//心跳数据
unsigned char HEART_LENGTH[2]={0x00,0x0B};
//数据上报
unsigned char UPDATA_LENGTH[2]={0x00,0x1D};
unsigned char PM25_DATA[3]={0x01,0x00,0x2F};
unsigned char PM03_DATA[3]={0x0E,0x00,0x14};
unsigned char TEM_DATA[3];
unsigned char HUM_DATA[3];
unsigned char POW_DATA[3];//电量的值,0X0A,0X00,0X64
unsigned char POW_sta[3];//电池充电状态 ，0x0F ,0X00,0X01

unsigned char datatemp[4];
//数据周期上报，PM2.5根据心跳上报数据
	Data_up_t data_up_t;
//处理PM2.5数据结构体
PM25_up_t pm25_up_t;
//温湿度结构体
SHT30_DATA_STRUCT   SHT30_data;

uint16_t  Sum_data(unsigned char buf[],int start,int stop){
	uint16_t sum_data=0;
	int i;
	for(i=start;i<=stop;i++){
		sum_data=sum_data+buf[i];
	}
	return sum_data;
}
//注册数据上报
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
	//输出端口配置PB1----pm2.5,PB12-----后端电源
	GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //使能PB端口时钟
	
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_12;//PB12作为检查是否有充电的信号端口
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //设置成上拉输入
 	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO_PB12
	
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;				 //pm25-->PB.1  PB12 端口配置
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;		 //IO口速度为2MHz
// GPIO_Init(GPIOB, &GPIO_InitStructure);	                                    				 //根据设定参数初始化GPIOB.1
// GPIO_ResetBits(GPIOB,GPIO_Pin_12);				 //PB12 输出0
	
 //控制PM25的开启和关闭
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;				 //pm25-->PB.1  PB1 端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;		 //IO口速度为2MHz
 GPIO_Init(GPIOB, &GPIO_InitStructure);	                                    				 //根据设定参数初始化GPIOB.1
 GPIO_ResetBits(GPIOB,GPIO_Pin_1);				 //PB.1输出0
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;				 //LED0-->PB.9 端口配置,输出控制电源开关机
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;		 //IO口速度为2MHz
 GPIO_Init(GPIOB, &GPIO_InitStructure);	                                    				 //根据设定参数初始化GPIOB.1
 GPIO_ResetBits(GPIOB,GPIO_Pin_9);				 //PB.9 输出0
	
}
void hal_board_init(void) {
	
	USART_GetFlagStatus(USART1, USART_FLAG_TC);
	USART_GetFlagStatus(USART2, USART_FLAG_TC);
	//读取flash中设备地址 4 byte
	STMFLASH_Read(FLASH_SAVE_ADDR,(u16*)DEV_MAC_ADDR,4);
	
	GPIO_output_Init();
	Adc_Init();		  		//ADC初始化
	//STMFLASH_Write(FLASH_SAVE_ADDR,(u16*)DEV_MAC_ADDR,4);
	SHT3X_Init();           //初始化I2C温湿度模块
	//send_rejest_data();
	PWER_ON_OFF=1;
	PM25_CON=1;
}
//心跳数据上报
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
	//ADC采集
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

	//获取ADC值
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
	POW_sta[2]=0x01;//充电状态
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
		//处理数据上报信息
		send_serial_data();
		sys_send_data_flag =0;
	}
//	if(sys_heart_flag==HEART_SERIAL_EVENT){
//		//心跳事件
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
	unsigned char up_check_buf[2];//命令返回校验数组
	
	unsigned char commend_up_data[64];//控制上传命令
	unsigned char commend_buf[24];//下发控制命令
	
	check_data=Sum_data(buf,2,len-5);
	check_buf[0]=check_data>>8;
	check_buf[1]=check_data;
	
	if(mymemcmp(buf+(len-4),check_buf,2)==0){
		
	if(port==HAL_UART2){
		//串口1用于通信，处理下发的控制命令
	  if(buf[4]==0x05){
			//控制命令
			mymemcpy(commend_buf,buf+17,len-21);
			//myuart_send(HAL_UART1,commend_buf,len-21);
			for(i=0;i<(len-21)/3;i=i+3){
				switch(commend_buf[i]){
					case 0x81:
						if(commend_buf[i+2]==0x0A){
							//开机

						}else if(commend_buf[i+2]==0x0B){
							//关机

						}
						break;
					case 0x82:
						if(commend_buf[i+2]==0x0C){
					
							//负离子开
						}else if(commend_buf[i+2]==0x0D){
							//负离子关
								
						}else if(commend_buf[i+2]==0x0E){
							//紫外灯开
								
						}else if(commend_buf[i+2]==0x0F){
							//紫外灯关
							
						}
						break;
					case 0x84:
						if(commend_buf[i+2]==0x0B){
							//待机
						}else if(commend_buf[i+2]==0x10){
							//1档风
								
						}else if(commend_buf[i+2]==0x11){
							//2档风
								
						}else if(commend_buf[i+2]==0x12){
							//3档风
								
						}else if(commend_buf[i+2]==0x13){
							//4挡风
								
						}
						break;
					case 0x85:
							if(commend_buf[i+2]==0x1A){
							//开启定时功能
						}else if(commend_buf[i+2]==0x1B){
							//关闭定时功能
						}else if(commend_buf[i+2]==0x47){
							//定时1小时自动关机
						}else if(commend_buf[i+2]==0x48){
							//定时2小时自动关机
						}else if(commend_buf[i+2]==0x49){
							//定时4小时自动关机
						}else if(commend_buf[i+2]==0x4A){
							//定时8小时自动关机
						}
						break;
					case 0x88:
							if(commend_buf[i+2]==0x1C){
							//定时一次有效
						}else if(commend_buf[i+2]==0x1D){
							//定时每天有效
						}else if(commend_buf[i+2]==0x1E){
							//定时周一有效
						}else if(commend_buf[i+2]==0x1F){
							//定时周二有效
						}else if(commend_buf[i+2]==0x20){
							//定时周三有效
						}else if(commend_buf[i+2]==0x21){
							//定时周四有效
						}else if(commend_buf[i+2]==0x22){
							//定时周五有效
						}else if(commend_buf[i+2]==0x23){
							//定时周六有效
						}else if(commend_buf[i+2]==0x24){
							//定时周天有效
						}
						break;
					case 0x92:
						if(commend_buf[i+2]==0x2C){
							//锁定/儿童锁开
							
						}else if(commend_buf[i+2]==0x2D){
							//解锁/儿童锁关
								
						}
						break;
					case 0x91:
						if(commend_buf[i+2]==0x3A){
							//开启设备，关闭其他模式
							
						}else if(commend_buf[i+2]==0x3B){
							//开启睡眠模式
								
						}else if(commend_buf[i+2]==0x3C){
							//开启手动模式
								
						}
						break;
					case 0x98:
						if(commend_buf[i+2]==0x40){
							//初效滤网初始化，更换滤网
							
						}else if(commend_buf[i+2]==0x41){
							//中效滤网初始化，更换滤网
						}else if(commend_buf[i+2]==0x42){
							//高效滤网初始化，更换滤网
						}
						break;
					default:
						break;		
				}
			}
			//返回控制命令
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
		//处理串口1接收的数据:命令格式EB 90 
		
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
//按键处理函数
void event_key_handle(void){
	if(key_long_down==1){
		//长按事件,关机
		PM25_CON=0;
		PWER_ON_OFF=0;
		RECEVE_HEART=0;
		key_long_down=0;
		//设置低功耗模式
   // hal_set_lowpower();
		
	}
	if(key_short_down==1){
		//短按按键事件
		PWER_ON_OFF=1;
		PM25_CON=1;
		RECEVE_HEART=1;
		key_short_down=0;
		//hal_wakeup_lowpower();
	}
	if(key_connect_down==1){
		//连按按键
		if(com_count==3){
		}
		key_connect_down=0;
	}
	
}
//串口处理回调函数
void uart_receve_handle(void)
{
	if(USART_RX_STA&0x8000)
		{	
			u8 uart_len;
			u8 uart_commandbuf[USART_REC_LEN];
			uart_len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
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
			uart2_len=USART2_RX_STA&0x3fff;//得到此次接收到的数据长度
			mymemcpy(uart2_commandbuf,USART2_RX_BUF,uart2_len);
			
//			if(mymemcmp(uart2_commandbuf+5,DEV_MAC_ADDR,4)==0){
//				
//			App_process_uart(HAL_UART2,uart2_commandbuf,uart2_len);
//			
//			}
			
				if((uart2_len==HAL_HEART_LENGTH)&(uart2_commandbuf[4]==0x01)){
					//收到串口蓝牙的心跳命令,发送数据
					RECEVE_HEART=1;
					send_serial_data();
				}
			
			if((uart2_len==HAL_MODIFY_LENGTH)&(uart2_commandbuf[4]==0xFD)){
					//修改设备ID
					Dev_dp_t dev_dp_t;//接收的数据
					Dev_up_t dev_up_t;//串口返回上报的数据
					uint16_t check_data;
					unsigned char check_buf[2];
					mymemcpy(dev_dp_t.data_buf,uart2_commandbuf,uart2_len);
				
					mymemcpy(dev_up_t.data_buf,uart2_commandbuf,uart2_len);	
			
					mymemcpy(DEV_MAC_ADDR,dev_dp_t.data_core.MAC_addr,4);	
					STMFLASH_Write(FLASH_SAVE_ADDR,(u16*)DEV_MAC_ADDR,4);	
					//返回设置地址数据
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

//串口处理回调函数
//void uart_receve_handle(void)
//{
//	if(USART_RX_STA&0x8000)
//		{	
//			u8 uart_len;
//			u8 uart_commandbuf[USART_REC_LEN];
//			uart_len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
//			mymemcpy(uart_commandbuf,USART_RX_BUF,uart_len);
//			
//			if(mymemcmp(uart_commandbuf+5,DEV_MAC_ADDR,4)==0){
//				
//			App_process_uart(HAL_UART1,uart_commandbuf,uart_len);
//			
//			}
//			if((uart_len==HAL_MODIFY_LENGTH)&(uart_commandbuf[4]==0xFD)){
//					//修改设备ID
//					Dev_dp_t dev_dp_t;//接收的数据
//					Dev_up_t dev_up_t;//串口返回上报的数据
//					uint16_t check_data;
//					unsigned char check_buf[2];
//					mymemcpy(dev_dp_t.data_buf,uart_commandbuf,uart_len);
//					mymemcpy(dev_up_t.data_buf,uart_commandbuf,uart_len);	
//			
//					mymemcpy(DEV_MAC_ADDR,dev_dp_t.data_core.MAC_addr,4);	
//					STMFLASH_Write(FLASH_SAVE_ADDR,(u16*)DEV_MAC_ADDR,4);	
//					//返回设置地址数据
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
//			uart2_len=USART2_RX_STA&0x3fff;//得到此次接收到的数据长度
//			mymemcpy(uart2_commandbuf,USART2_RX_BUF,uart2_len);
//			myuart_send(HAL_UART2,uart2_commandbuf,uart2_len);
//			USART2_RX_STA=0;
//			mymemset(USART2_RX_BUF,0,USART_REC_LEN);
//			mymemset(uart2_commandbuf,0,USART_REC_LEN);
//		}		
//}


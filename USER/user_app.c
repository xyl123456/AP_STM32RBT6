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
 

//注册信息数据
unsigned char HEARD_DATA[2]={0xEB,0x90};
unsigned char HEARD_LENGTH[2]={0x00,0x14};
u16 DEV_MAC_ADDR[4]={0x00,0x00,0x00,0x00};
unsigned char VERSION[3]={0x00,0x00,0x01};
unsigned char HARD_VERSION[3]={0x00,0x00,0x01};
unsigned char HEART_TIME[3]={0x00,0x00,0x1E};
unsigned char TAIL_DATA[2]={0x0D,0x0A};

//wifi要求下发注册命令
u8 WIFI_CMD[13]={0xEB,0x90,0x00,0x0B,0x0F,0x00,0x00,0x00,0x00,0x00,0x1A,0x0D,0x0A};
//心跳数据
unsigned char HEART_LENGTH[2]={0x00,0x0B};
//数据上报
unsigned char UPDATA_LENGTH[2]={0x00,0x1D};
unsigned char PM25_DATA[3]={0x01,0x00,0x28};
unsigned char PM03_DATA[3]={0x0E,0x00,0x28};
unsigned char TEM_DATA[3];
unsigned char HUM_DATA[3];
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
 	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);	 //使能PB端口时钟
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_5;//PA5作为输入，检测是否wifi配置完成
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //设置成浮空输入
 	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO_PA5
	
	//GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_12;//PB12作为检查是否有充电的信号端口
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //设置成上拉输入
 	//GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO_PB12
	
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;				 //pm25-->PB.1  PB12 端口配置
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;		 //IO口速度为2MHz
// GPIO_Init(GPIOB, &GPIO_InitStructure);	                                    				 //根据设定参数初始化GPIOB.12
// GPIO_ResetBits(GPIOB,GPIO_Pin_12);				 //PB12 输出0
	
 //控制PM25的开启和关闭
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;				 //pm25-->PB.1  PB1 端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;		 //IO口速度为2MHz
 GPIO_Init(GPIOB, &GPIO_InitStructure);	                                    				 //根据设定参数初始化GPIOB.1
 GPIO_SetBits(GPIOB,GPIO_Pin_1);				 //PB.1输出1,默认上电打开PM2.5
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;				 //PB.9 端口配置,输出控制wifi模块的配置模式
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;		 //IO口速度为2MHz
 GPIO_Init(GPIOB, &GPIO_InitStructure);	                                    				 //根据设定参数初始化GPIOB.9
 GPIO_SetBits(GPIOB,GPIO_Pin_9);				 //PB.9 输出1，低电平表示进入配置模式
	
}
void hal_board_init(void) {
	GPIO_output_Init();
	
	USART_GetFlagStatus(USART1, USART_FLAG_TC);
	USART_GetFlagStatus(USART2, USART_FLAG_TC);
	//读取flash中设备地址 4 byte
	STMFLASH_Read(FLASH_SAVE_ADDR,(u16*)DEV_MAC_ADDR,4);
	
	
	//Adc_Init();		  		//ADC初始化
	//STMFLASH_Write(FLASH_SAVE_ADDR,(u16*)DEV_MAC_ADDR,4);
	SHT3X_Init();           //初始化I2C温湿度模块
	//send_rejest_data();
	//PWER_ON_OFF=1;//用于控制开机电路的IO口
}
//心跳数据上报
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
	
			temp_buffer=(int)SHT30_data.Temp+0x5E;//原本加100，考虑芯片和板子的温差，加了94
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
			//当wifi连接上AP后才会发送数据
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
	unsigned char up_check_buf[2];//命令返回校验数组
	
	unsigned char commend_up_data[64];//控制上传命令
	unsigned char commend_buf[24];//下发控制命令,最大支持8组命令同时下发
	
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
						PM25_CON=1;
						}else if(commend_buf[i+2]==0x0B){
							//关机
						PM25_CON=0;
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
			
			if(mymemcmp(uart2_commandbuf+5,DEV_MAC_ADDR,4)==0){
				
				if(mymemcmp(uart2_commandbuf,(void *)HEARD_DATA,2)==0){
						App_process_uart(HAL_UART2,uart2_commandbuf,uart2_len);
					}
				}
			if(mymemcmp(uart2_commandbuf,(void *)WIFI_CMD,13)==0)
			{
				send_rejest_data();//wifi连接注册信息
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
		
			USART2_RX_STA=0;
			uart2_len=0;
			mymemset(USART2_RX_BUF,0,USART_REC_LEN);
			mymemset(uart2_commandbuf,0,USART_REC_LEN);
			
		}
	
}


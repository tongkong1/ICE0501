#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>
#include "hw_memmap.h"
#include "debug.h"
#include "gpio.h"
#include "hw_i2c.h"
#include "hw_types.h"
#include "i2c.h"
#include "pin_map.h"
#include "sysctl.h"
#include "systick.h"
#include "interrupt.h"
#include "uart.h"
#include "hw_ints.h"
#include "string.h"
#include "pwm.h"

#define SYSTICK_FREQUENCY		1000			//1000hz

#define	I2C_FLASHTIME				100				//100mS
#define GPIO_FLASHTIME			300				//300mS

#define   FASTFLASHTIME			(uint32_t)500000
#define   SLOWFLASHTIME			(uint32_t)1000000
//*****************************************************************************
//
//I2C GPIO chip address and resigster define
//
//*****************************************************************************
#define TCA6424_I2CADDR 					0x22
#define PCA9557_I2CADDR						0x18

#define PCA9557_INPUT							0x00
#define	PCA9557_OUTPUT						0x01
#define PCA9557_POLINVERT					0x02
#define PCA9557_CONFIG						0x03

#define TCA6424_CONFIG_PORT0			0x0c
#define TCA6424_CONFIG_PORT1			0x0d
#define TCA6424_CONFIG_PORT2			0x0e

#define TCA6424_INPUT_PORT0				0x00
#define TCA6424_INPUT_PORT1				0x01
#define TCA6424_INPUT_PORT2				0x02

#define TCA6424_OUTPUT_PORT0			0x04
#define TCA6424_OUTPUT_PORT1			0x05
#define TCA6424_OUTPUT_PORT2			0x06


void 		Delay(uint32_t value);
void 		S800_GPIO_Init(void);
uint8_t 	I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData);
uint8_t 	I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr);
void		S800_I2C0_Init(void);
void 		S800_UART_Init(void);
void    PWM_Init(void);

//systick software counter define
volatile uint16_t systick_10ms_couter,systick_100ms_couter,systick_1s_couter;
volatile uint8_t	systick_10ms_status,systick_100ms_status,systick_1s_status;


volatile uint8_t result,cnt,key_value,gpio_status;
volatile uint8_t rightshift = 0x01;
volatile uint8_t rightshift1 = 0x01;
uint32_t ui32SysClock;
uint8_t seg7[] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c,0x58,0x5e,0x79,0x71,0x40};  //��С����0-F
uint8_t seg_7[] = {0xbf,0x86,0xdb,0xcf,0xe6,0xed,0xfd,0x87,0xff,0xef,0xf7,0xfc,0xb9,0xde,0xf9,0xf1};   //��С����0-F
uint8_t led7[] = {0x7f,0xbf,0xdf,0xef,0xf7,0xfb,0xfd,0xfe};
uint8_t ledlight[] = {0xfe,0xfd,0xfb,0xf7,0xef,0xdf,0xbf,0x7f};   //led��1-8

uint8_t uart_receive_char;
uint8_t hour=12,minute=00,second=00,year1=20,year=23,mon=06,day=12,alarm_h=00,alarm_m=00,alarm_s=00;    //ʱ���롢�����ճ�ʼֵ
uint8_t cd_s=10,cd_ms=50;           //����ʱ��ֵ

int cdm=0,cds=0,cdms=0; 

int func = 1;								//����ѡ��
int alarm_enable = 1;       //����ʹ��
int cd_enable = 0;          //����ʱʹ��

int month[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
uint32_t opening[10]={8511,10121,11364,12755,14316,15174,17021,19120,45454}; //����


uint32_t delay_time, now_key_value = 1, last_key_value = 1;//USER_SW1
uint32_t delay_time2, now_key_value2 = 1, last_key_value2 = 1;//USER_SW2

uint32_t second_pressed = 0,second_released=0,m_second_pressed = 0,m_second_released=0;
uint32_t second_continue = 0,m_second_continue=0;

uint32_t second_pressed2 = 0,second_released2 = 0,m_second_pressed2 = 0,m_second_released2 = 0;
uint32_t second_continue2 = 0,m_second_continue2 = 0;

char time_record[256]; //��������ַ���
char date_record[256]; //��������ַ���
char alarm_record[256]; //��������ַ���
char CountDown_record[256];
char RxBuf[256];            //�ַ������

char func_print[256];  //���������л�ʱ�����������ʾ

int cnt1 = 0;  //PWM������ѭ����������


void reset(void); 						 //ϵͳ����
void TIMEdisplay(void);        //�������ʾʱ��
void DATEdisplay(void);        //�������ʾ����
void ALARMdisplay(void);       //�������ʾ����
bool push(int order);		       //���SW1-SW8�����Ƿ���
void elevation(void);          //��ʱ��λ
void soundPWM(int tone);       //���Ʒ���������
void alarm(void);              //��������
void alarm_music_light(void);  //��������
void CountDown_display(void);  //����ʱ��ʾ
void CDset(void);              //����ʱ����
void CountDown(void);          //����ʱ����
void cd_music_light(void);     //����ʱ����


void UARTStringPutNonBlocking(const char *cMessage);


void Function1(void); //ģʽ1����ʾ���ڣ�������
void Function2(void); //ģʽ2���޸����ڣ�������
void Function3(void); //ģʽ3����ʾʱ�䣬ʱ����
void Function4(void); //ģʽ4���޸�ʱ�䣬ʱ����
void Function5(void); //ģʽ5������
void Function6(void); //ģʽ6������ʱ

void User_PressButton(void);  //��尴��USER_SW1
void User_PressButton2(void); //��尴��USER_SW2

int main(void)
{
	volatile uint16_t	i2c_flash_cnt,gpio_flash_cnt;
	//use internal 16M oscillator, PIOSC
  //ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT |SYSCTL_USE_OSC), 16000000);		
	//ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT |SYSCTL_USE_OSC), 8000000);		
	//use external 25M oscillator, MOSC
   //ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN |SYSCTL_USE_OSC), 25000000);		

	//use external 25M oscillator and PLL to 120M
   //ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 120000000);;		
	ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT | SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 16000000);
	
	SysTickPeriodSet(ui32SysClock/SYSTICK_FREQUENCY);    //0.001��
	SysTickEnable();
	SysTickIntEnable();																		//ʹ��Systick�ж�
	  
	IntMasterDisable();	//���ж�

	S800_GPIO_Init();
	S800_I2C0_Init();
	S800_UART_Init();
	PWM_Init();
	
	
	
	IntEnable(INT_UART0);
  UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);	//ʹ�� UART0 RX,TX
  IntMasterEnable();		//���ж�
  
  reset();
	
	while(1)
  {
		/*
		//�Ƶ��жϳ����У���������ʱʱ�ӿɼ�������
		if (func != 4 && systick_1s_status)
		{
			systick_1s_status	= 0;
			
			second++;
			elevation();	
		}
		*/
		
		
		//ͨ������1�л���ͬģʽ
		if(push(1)) { 
			func=func%6+1;
			
			//�ڴ�����ʾ��ǰ�����ĸ�ģʽ
			sprintf(func_print,"\r\n");
			sprintf(func_print,"%8s%02u","function=",func); 
			UARTStringPutNonBlocking(func_print);
			sprintf(func_print,"\r\n");
			UARTStringPutNonBlocking(func_print);
			
		}
		
		alarm(); //���ӹ���

		switch(func)
	   {
				case 1:    //ģʽ1����ʾ���ڣ�������
					{
						Function1();
						break;
					}
				case 2:    //ģʽ2����������
					{								
						Function2();
						
						break;
					}	
				case 3:    //ģʽ3��ʱ����ʾ
					{ 
						Function3();
						User_PressButton();
						User_PressButton2();
						break;			
					}
				case 4:    //ģʽ4��ʱ������
					{						
						Function4();
					  break;
					}
				case 5:    //ģʽ5��������ʾ������
					{
						Function5();
						break;
					}
				case 6:    //ģʽ6������ʱ�����ʾ������
					{
						Function6();						
						break;
					}
	   }//switch����		
	 
	}//while(1)����
	
}

//ģʽ1����ʾ���ڣ�������
void Function1(void) 
{
	
	DATEdisplay();          //�������ʾ��ǰ����
	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,ledlight[func-1]);	
	Delay(100);

}

//ģʽ2���޸����ڣ�������
void Function2(void)
{
	DATEdisplay();
	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,ledlight[func-1]); 
	Delay(100);

	if(push(3) && mon!=01)   //����SW3�·ݼ�
	{
		int m = mon; //����ֱ��mon--����������Ч������ͬ
		m--;
		mon = m;
	}			
	
	DATEdisplay();
	Delay(100);

	if(push(2))   //����SW2��ݼ�
	{
		if(year!=00){
			int y = year;
			y--;
			year = y;
		}
		else{
		  year=99;
			year1-=1;
		}
		
	}		
	
	DATEdisplay(); 						
	Delay(100);
	
	if(push(4) && day!=01)   //����SW4���ڼ�
	{
		int d = day;
		d--;
		day = d;
	}
	
	DATEdisplay();
	Delay(100);
	
	if(push(7))   //����SW7�����
	{
	  
		if(year!=99){
			year++;			
		}
		else{
		  year=00;
			year1+=1;
		}			
	}
	
	DATEdisplay();
	Delay(100);

	while(push(6))   //����SW6�·���
		 mon++;	
	
	DATEdisplay();
	Delay(100);					
	
	while(push(5))   //����SW5������
		 day++;
	DATEdisplay();
	Delay(100);				

} 

//ģʽ3����ʾʱ�䣬ʱ����
void Function3(void) 
{
	TIMEdisplay();                   //�������ʾ��ǰʱ��
	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,ledlight[func-1]);
	Delay(100);
						
}

//ģʽ4���޸�ʱ�䣬ʱ����
void Function4(void) 
{
	TIMEdisplay();
	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,ledlight[func-1]);
	Delay(100);
	
	if(push(2) && hour!=00)   //����SW2Сʱ��
	{
		int h = hour;
		h--;
		hour = h;
	}
	
	TIMEdisplay();	
	Delay(100);						
	
	if(push(3) && minute!=00)   //����SW3���Ӽ�
	{
		int m = minute;
		m--;
		minute = m;
	}			
	
	TIMEdisplay();	
	Delay(100);								
	
	if(push(4) && second!=00)   //����SW4���
	{
		int s = second;
		s--;
		second = s;
	}
	
	TIMEdisplay();	
	Delay(100);		
	
	while(push(7))   //����SW7Сʱ��
	{
		hour++;	
		elevation();
	}
	
	TIMEdisplay();	
	Delay(100);		
	
	while(push(6))   //����SW6������
	{
		minute++;	
		elevation();
	}

	TIMEdisplay();	
	Delay(100);		
	
	while(push(5))   //����SW5����
	{
		second++;	
		elevation();
	}
	
	TIMEdisplay();	
	Delay(100);		

}

//ģʽ5������
void Function5(void) 
{
	ALARMdisplay();
	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,ledlight[func-1]);	
	Delay(100);

	if(push(2) && alarm_h!=00)   //����SW2Сʱ��
	{
		int h = alarm_h;
		h--;
		alarm_h = h;
	}

	ALARMdisplay();						
	Delay(100);	

	if(push(3) && alarm_m!=00)   //����SW3���Ӽ�
	{
		int m = alarm_m;
		m--;
		alarm_m = m;
	}				

	ALARMdisplay();						
	Delay(100);	
											
	if(push(4) && alarm_s!=00)   //����SW4���
	{
		int s = alarm_s;
		s--;
		alarm_s = s;
	}

	ALARMdisplay();						
	Delay(100);	
							
	while(push(7))   //����SW7Сʱ��
	{
		alarm_h++;	
		elevation();
	}

	ALARMdisplay();						
	Delay(100);	
						
	while(push(6))   //����SW6������
	{
		alarm_m++;	
		elevation();
	}

	ALARMdisplay();						
	Delay(100);	
							
	while(push(5))   //����SW5����
	{
		alarm_s++;	
		elevation();
	}	

	ALARMdisplay();						
	Delay(100);								

}

//ģʽ6������ʱ
void Function6(void) 
{
	if(func == 6 && cd_enable)
	{
		if (systick_10ms_status)
			{
				systick_10ms_status	= 0;
				
				cd_ms--; //ÿ0.01s��1
			}
			CountDown();	//���µ�ǰ����ʱ							
	}	

	CountDown_display();  //�������ʾ
	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,ledlight[func-1]);  
	Delay(100);						

	CDset(); 
	CountDown_display();
	Delay(100);
	if(push(7)) cd_enable = cd_enable%2+1;   //����ʱʹ��
	
}

//��尴��USER_SW1
void User_PressButton(void)  
{
	//
	last_key_value = now_key_value;			//��¼��һ�ΰ�����ֵ
	//SysCtlDelay( ui32SysClock / 150);	//����20ms���ٶ�ȡ��������ʱ����

	now_key_value = GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0);	//��ȡ��ʱ�İ���ֵ
	delay_time = SLOWFLASHTIME;
	
	if (now_key_value == 0 && last_key_value == 1)	//���жϰ������º�
	{
		sprintf(func_print,"\r\n");
		UARTStringPutNonBlocking(func_print);
		sprintf(time_record,"%9s%02u%s%03u","PRESSTIME",second,".",cdms);
		second_pressed = second;
		m_second_pressed = cdms;
		UARTStringPutNonBlocking(time_record);
		sprintf(func_print,"\r\n");
		UARTStringPutNonBlocking(func_print);
	}
	
	if (now_key_value == 1 && last_key_value == 0)	//���жϰ����ɿ�
	{
		sprintf(func_print,"\r\n");
		UARTStringPutNonBlocking(func_print);
		sprintf(time_record,"%11s%02u%s%03u","RELEASETIME",second,".",cdms);
		second_released = second;
		m_second_released = cdms;
		UARTStringPutNonBlocking(time_record);
		sprintf(func_print,"\r\n");
		UARTStringPutNonBlocking(func_print);
		
		if(m_second_released<m_second_pressed){
			m_second_continue = 1000 + m_second_released - m_second_pressed;
			second_continue = second_released - second_pressed - 1;
		}
		else{
			m_second_continue = m_second_released - m_second_pressed;
			second_continue = second_released - second_pressed;
		}
		sprintf(func_print,"\r\n");
		UARTStringPutNonBlocking(func_print);
		sprintf(time_record,"%12s%02u%s%03u","CONTINUETIME",second_continue,".",m_second_continue);						
		UARTStringPutNonBlocking(time_record);
		sprintf(func_print,"\r\n");
		UARTStringPutNonBlocking(func_print);		
	}						 
}

//��尴��USER_SW2
void User_PressButton2(void)  
{
	//
	last_key_value2 = now_key_value2;			//��¼��һ�ΰ�����ֵ
	//SysCtlDelay( ui32SysClock / 150);	//����20ms���ٶ�ȡ��������ʱ����

	now_key_value2 = GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_1);	//��ȡ��ʱ�İ���ֵ
	delay_time2 = SLOWFLASHTIME;
	
	if (now_key_value2 == 0 && last_key_value2 == 1)	//���жϰ������º�
	{
		sprintf(time_record,"%9s%02u%s%03u","PRESSTIME",second,".",cdms);
		second_pressed2 = second;
		m_second_pressed2 = cdms;
		UARTStringPutNonBlocking(time_record);
		sprintf(func_print,"\r\n\r\n");
		UARTStringPutNonBlocking(func_print);
	}
	
	if (now_key_value2 == 1 && last_key_value2 == 0)	//���жϰ����ɿ�
	{
		sprintf(time_record,"%11s%02u%s%03u","RELEASETIME",second,".",cdms);
		second_released2 = second;
		m_second_released2 = cdms;
		UARTStringPutNonBlocking(time_record);
		sprintf(func_print,"\r\n\r\n");
		UARTStringPutNonBlocking(func_print);
		
		if(m_second_released2 < m_second_pressed2){
			m_second_continue2 = 1000 + m_second_released2 - m_second_pressed2;
			second_continue2 = second_released2 - second_pressed2 - 1;
		}
		else{
			m_second_continue2 = m_second_released2 - m_second_pressed2;
			second_continue2 = second_released2 - second_pressed2;
		}
		sprintf(time_record,"%12s%02u%s%03u","CONTINUETIME",second_continue2,".",m_second_continue2);						
		UARTStringPutNonBlocking(time_record);
		sprintf(func_print,"\r\n\r\n");
		UARTStringPutNonBlocking(func_print);		
	}						 
}

//���Ʒ���������
void soundPWM(int tone)      
{
	
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, tone);

	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7,tone/2);
	
	PWMOutputState(PWM0_BASE,PWM_OUT_7_BIT,true);

}

//����
void reset(void)                     
{
	int i = 0,j = 0,t = 0,temp = 0;	
	int ID[8]={2,1,9,1,0,4,0,4};
	
	hour=12, minute=00, second=00, year1=20, year=23, mon=06, day=12, alarm_h=12, alarm_m=02, alarm_s=00, alarm_enable = 1, cd_s=10, cd_ms=50, cd_enable = 0; //��������
	
	for(cnt1 = 0; cnt1 < 11; cnt1++)
	{
	uint8_t family[50]={7,5,2,5,6,8,6,7,6,2,5};
	int bat[50] = {1000000,1000000,1000000,1000000,1000000,2000000,1000000,1000000,1000000,1000000,1800000};
	soundPWM(opening[family[cnt1]]);
	Delay(bat[cnt1]);
	}
	PWMOutputState(PWM0_BASE,PWM_OUT_7_BIT,false);			//���ܶ�Ӧ�˿�
	
	
	for(t=0;t<=15;t++){
		//����������
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[ID[temp]]);	//write port 1 				
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<temp);	//write port 2
		
		//LED�����
		result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~(1<<temp));	

		temp = (temp+1) % 8;

		//PF0��˸
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, ~GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0));		// Turn over the PF0 
		SysCtlDelay(ui32SysClock/9); //��ʱ
	}
	
}

//SW1-SW8�������
bool push(int order)       
{
	if(I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0) == 0xff)	
			return false;			
			
		if(I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0) == led7[8-order])				//��ⰴ���Ƿ���
		{	
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);	
			Delay(1000000);       //�ӳ�����
			while(I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0) == led7[8-order])
			GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_0,GPIO_PIN_0);			//���°���ʱ��PF0����
			Delay(1000);
			GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_0,0x0);
			
			return true;
		}
		return false;				//���µİ�����Ŀ�갴����ƥ�䣬����false
}

//ʱ����ʾ
void TIMEdisplay(void)   
{
	int i = 0;
	if(rightshift == 0x01)   i = hour/10;
	if(rightshift == 0x02)   i = hour%10;
	if(rightshift == 0x08)   i = minute/10;
	if(rightshift == 0x10)   i = minute%10;
	if(rightshift == 0x40)   i = second/10;
	if(rightshift == 0x80)   i = second%10;
	if(rightshift == 0x04 || rightshift==0x20)   i = 16; //�����̺���
	
	  result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[i]);	//write port 1 				
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,rightshift);	//write port 2
	
	if(rightshift != 0x80)
	    rightshift = rightshift<<1;
  else 
			rightshift= 0x01;
}

//������ʾ
void DATEdisplay(void)     
{
	int i = 0;
	
	
	if(rightshift == 0x01)   i = year1/10;
	if(rightshift == 0x02)   i = year1%10;
	if(rightshift == 0x04)   i = year/10;
	if(rightshift == 0x08)   i = year%10;
	if(rightshift == 0x10)   i = mon/10;
	if(rightshift == 0x20)   i = mon%10;
	if(rightshift == 0x40)   i = day/10;
	if(rightshift == 0x80)   i = day%10;

	
	  result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[i]);	//write port 1 				
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,rightshift);	//write port 2
	
	if(rightshift != 0x80)
	    rightshift = rightshift<<1;
  else 
			rightshift= 0x01;
}

//������ʾ
void ALARMdisplay(void)   
{
	int i = 0;
	if(rightshift == 0x01)   i = alarm_h/10;
	if(rightshift == 0x02)   i = alarm_h%10;
	if(rightshift == 0x08)   i = alarm_m/10;
	if(rightshift == 0x10)   i = alarm_m%10;
	if(rightshift == 0x40)   i = alarm_s/10;
	if(rightshift == 0x80)   i = alarm_s%10;
	if(rightshift == 0x04 || rightshift==0x20)   i = 16;
	
	  result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[i]);	//write port 1 				
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,rightshift);	//write port 2
	
	if(rightshift != 0x80)
	    rightshift = rightshift<<1;
  else 
			rightshift= 0x01;
}

//�������к���
void alarm(void)       
{
		if(push(8)) alarm_enable = alarm_enable%2+1;   //����ʹ��
		if(alarm_enable == 1)
				{
						result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,ledlight[7]);     //LED8��������������ڹ���	
						if(hour==alarm_h && minute==alarm_m && second==alarm_s)
						{
							while(!push(8))
							{	
								alarm_music_light();
						  	if(push(8)) break;    //�ص�����
							}				
						}
				}
}

//��������&ʱ����ˮ��
void alarm_music_light(void)     
{
	for(cnt1 = 0; cnt1 < 8; cnt1++)
	{
	uint8_t alarm[50]={1,3,5,3,4,3,2,1};
	int bat[50] = {1000000,1000000,1000000,1000000,1000000,1000000,1000000,1000000};
	TIMEdisplay();
	soundPWM(opening[alarm[cnt1]]);
	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,ledlight[cnt1]);	
	Delay(bat[cnt1]);
	}
	PWMOutputState(PWM0_BASE,PWM_OUT_7_BIT,false);			//���ܶ�Ӧ�˿�
}

//����ʱ��ʾ
void CountDown_display(void)   
{
	if(rightshift == 0x01)
	{
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg_7[12]);	//write port 1 				
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,rightshift);	//write port 2
	}
	if(rightshift == 0x02) 
	{
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg_7[13]);	//write port 1 				
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,rightshift);	//write port 2
	}

	if(rightshift == 0x10)
	{
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[cd_s/10]);	//write port 1 				
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,rightshift);	//write port 2
	}
	if(rightshift == 0x20)
	{
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg_7[cd_s%10]);	//write port 1 				
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,rightshift);	//write port 2
	}
	if(rightshift == 0x40)
	{
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[cd_ms/10]);	//write port 1 				
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,rightshift);	//write port 2
	}
	if(rightshift == 0x80)
	{
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[cd_ms%10]);	//write port 1 				
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,rightshift);	//write port 2
	} 
	
	if(rightshift != 0x80)
	    rightshift = rightshift<<1;
  else 
			rightshift= 0x01;
}

//����ʱ����
void CDset(void)            
{
	        if(push(3) && cd_s!=00)   //���
						{
						int s = cd_s;
							s--;
							cd_s = s;
						}				
           			
			    if(push(4) && cd_ms!=00)   //0.01���
						{
						int ms = cd_ms;
							ms--;
							cd_ms = ms;
						}
						

           if(push(6))   //����
						{
							cd_s++;	
						  elevation();
						}
						
           if(push(5))   //0.01����
						{
						  cd_ms++;	
					    elevation();
						}	
}	

//����ʱ���£������ж�
void CountDown(void)   
{
	if(cd_ms == 00 && cd_s!= 00)
	{
		cd_s--;
		cd_ms = 99;
	}
	if(cd_ms == 00 && cd_s == 00)
		{
			cd_music_light();
			cd_enable = 0;
			
			//�ٴγ�ʼ��
			cd_s=10;
			cd_ms=50;
	  }
}

//����ʱ����&��ˮ��
void cd_music_light(void)     
{
	for(cnt1 = 0; cnt1 < 26; cnt1++)
	{
	uint8_t alarm[50]={3,2,1,2,3,3,3,2,2,2,3,5,5,3,2,1,2,3,3,3,1,2,2,3,2,1};
	int bat[50] = {400000,400000,400000,400000,400000,400000,800000,400000,400000,800000,400000,400000,800000,400000,400000,400000,400000,400000,400000,400000,400000,400000,400000,400000,400000,1000000};
	soundPWM(opening[alarm[cnt1]]);
	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,ledlight[0]);
	Delay(40000);
  result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,ledlight[1]);
	Delay(40000);
  result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,ledlight[2]);	
	Delay(40000);
  result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,ledlight[3]);	
	Delay(40000);
  result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,ledlight[4]);	
	Delay(40000);
  result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,ledlight[5]);	
	Delay(40000);
  result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,ledlight[6]);	
	Delay(40000);
  result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,ledlight[7]);	
	Delay(40000);
		
	Delay(bat[cnt1]);
	PWMOutputState(PWM0_BASE,PWM_OUT_7_BIT,false);			//���ܶ�Ӧ�˿�
	Delay(4000);
	}

}

//Delay
void Delay(uint32_t value)
{
	uint32_t ui32Loop;
	for(ui32Loop = 0; ui32Loop < value; ui32Loop++){};
}

void UARTStringPut(uint8_t *cMessage)
{
	while(*cMessage!='\0')
		UARTCharPut(UART0_BASE,*(cMessage++));
}

void UARTStringPutNonBlocking(const char *cMessage)
{
	while(*cMessage!='\0')
	{
		if (UARTSpaceAvail(UART0_BASE))
		  UARTCharPutNonBlocking(UART0_BASE,*(cMessage++));
	}
}


void S800_UART_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);						//Enable PortA
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));			//Wait for the GPIO moduleA ready

	GPIOPinConfigure(GPIO_PA0_U0RX);												// Set GPIO A0 and A1 as UART pins.
  GPIOPinConfigure(GPIO_PA1_U0TX);    			

  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Configure the UART for 115,200, 8-N-1 operation.
  UARTConfigSetExpClk(UART0_BASE, ui32SysClock,115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE));
	UARTStringPut((uint8_t *)"\r\nHEY!WELCOME!\r\n");
	
	//
	UARTStringPut("\r\n");UARTStringPut("\r\n");
	UARTStringPut("00000000 000000000000000000000000000000 000000000000 000 0000000\r\n");
	UARTStringPut("00000000 000000000 00000 000000000 0000 0000 0000000 000 0000000\r\n");
	UARTStringPut("      00 0000000000 00  00    00000 000 0000 0000000 000 0000000\r\n");
	UARTStringPut("00000 00      00000 00 000 00 000000 00 000 00000 00 00        0\r\n");
	UARTStringPut("00000 0 00000 00000000 000 00 000000 00 00 000000 00 00 0 000000\r\n");
	UARTStringPut("0 00 00 0000 000000000 000 00 000000000 000000000 00 0 000 00000\r\n");
	UARTStringPut("00 0 0 00 000000    00 000 00 00               00 00  0000 00000\r\n");
	UARTStringPut("000 0 000 000000000 00 000 00 0000000 000 0000000 00 00000000000\r\n");
	UARTStringPut("000 00000 000000000 00 000 00 0000000 000 0000000 00 00       00\r\n");
	UARTStringPut("00 0 000 0 00000000 00 0 0  0 0000000 000 0000000 00 00 00 00 00\r\n");
	UARTStringPut("00100 00 0 00000000 00  00 0 00000000 000 0000000 00 00 00 00 00\r\n");
	UARTStringPut("0 000 0 000 0000000 00 000 000000000 0000 0000 00 00 00 00 00 00\r\n");
	UARTStringPut(" 000000 000 0000000 000000 000000000 0000 0000 00 00 00 00 00 00\r\n");
	UARTStringPut("000000 00000 00000 0 00000 00000000 00000 0000 00000 00       00\r\n");
	UARTStringPut("00000 0000000 000 000          000 0000000     00000 00 00000 00\r\n");
	UARTStringPut("0000 000000000 00000000000000000  000000000000000000 00000000000\r\n");
	UARTStringPut("\r\n");UARTStringPut("\r\n");
		
	UARTStringPut((uint8_t *)"\r\nYou can enter [HELP] or ? for the help menu.\r\n");
	UARTFIFOLevelSet(UART0_BASE,UART_FIFO_TX2_8,UART_FIFO_RX7_8);
}
void S800_GPIO_Init(void)
{
	
	
	/*
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);						//Enable PortF
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));			//Wait for the GPIO moduleF ready

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);						//Enable PortJ	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));			//Wait for the GPIO moduleJ ready	
	
   GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);			//Set PF0 as Output pin

	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0);			//Set PJ0 as input pin
	GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);			//Set PF1 as Output pin

	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_1);			//Set PJ1 as input pin
	GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	
	*/
	
	
	
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);						//Enable PortF
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));			//Wait for the GPIO moduleF ready
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);						//Enable PortJ	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));			//Wait for the GPIO moduleJ ready	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);						//Enable PortN	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));			//Wait for the GPIO moduleN ready		
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);						//Enable PortJ	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK));			//Wait for the GPIO moduleK ready	
	
	
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);			//Set PF0 as Output pin
  GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);			//Set PN0,PN1 as Output pin
	GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_5);
	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1);//Set the PJ0,PJ1 as input pin
	GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
}

void S800_I2C0_Init(void)
{
	uint8_t result;
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);
  GPIOPinConfigure(GPIO_PB3_I2C0SDA);
  GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
  GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

	I2CMasterInitExpClk(I2C0_BASE,ui32SysClock, true);										//config I2C0 400k
	I2CMasterEnable(I2C0_BASE);	

	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT0,0x0ff);		//config port 0 as input
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT1,0x0);			//config port 1 as output
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT2,0x0);			//config port 2 as output 

	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_CONFIG,0x00);					//config port as output
	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0ff);				//turn off the LED1-8
	
}

void PWM_Init(void)             //��������ʼ��
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);

  GPIOPinConfigure(GPIO_PK5_M0PWM7);

	GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_5);
  PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN |PWM_GEN_MODE_NO_SYNC);	
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 20000);	
	
  PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7,20000/2);
	PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, false);
	PWMGenEnable(PWM0_BASE, PWM_GEN_3);
}


uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData)
{
	uint8_t rop;
	while(I2CMasterBusy(I2C0_BASE)){};
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while(I2CMasterBusy(I2C0_BASE)){};
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);

	I2CMasterDataPut(I2C0_BASE, WriteData);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
	while(I2CMasterBusy(I2C0_BASE)){};

	rop = (uint8_t)I2CMasterErr(I2C0_BASE);
	return rop;
}

uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr)
{
	uint8_t value,rop;
	while(I2CMasterBusy(I2C0_BASE)){};	
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);
//	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);		
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_SEND);
	while(I2CMasterBusBusy(I2C0_BASE));
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);
	Delay(1);
	//receive data
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, true);
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);
	while(I2CMasterBusBusy(I2C0_BASE));
	value=I2CMasterDataGet(I2C0_BASE);
		Delay(1);
	return value;
}

/*
	Corresponding to the startup_TM4C129.s vector table systick interrupt program name
*/
void elevation(void)   //��ʱ��λ
{
	int j = mon;

if(cd_ms>99)
	{
		cd_s++;
		cd_ms = 00;
	}
	
	if(cd_s>=100)
	{
		cd_s = 99;
	}
	
  if(second>=60)
	{
		minute++;
		second=0;
	}
	
	if(minute>=60)
	{
		hour++;
		minute=0;
	}
	
	if(hour>=24)	
	{
		day++;
		hour=0;
	}
	
	if (((year+year1*100)%4==0&&(year+year1*100)%100!=0)||(year+year1*100)%400==0) month[1]=29; //����
	if(month[j-1] < day)
	{
		day=01;
	  mon++;
	}
	
	if(mon > 12)
	{
	  mon=01;
		year++;
	}
	
	if(year >= 100)
	{
		year=00;
		year1++;
	}
		
	
	
	
}

void SysTick_Handler(void) //1ms��SysTick
{
		if (systick_100ms_couter == 0) //����1ms��SysTick����100ms�Ķ�ʱ��
	{
		systick_100ms_couter = 100;
		systick_100ms_status = 1;
	}
	else
		systick_100ms_couter--;
	
	if (systick_10ms_couter	== 0) //����1ms��SysTick����10ms�Ķ�ʱ��
	{
		systick_10ms_couter	 = 10;
		systick_10ms_status  = 1;
	}
	else
		systick_10ms_couter--;
	
	if (systick_1s_couter	== 0) //����1ms��SysTick����1s�Ķ�ʱ��
	{
		systick_1s_couter	 = 1000;
		systick_1s_status  = 1;
	}
	else
		systick_1s_couter--;
	
	//����
	cdms++;
	if(cdms>=1000){
	  cdms -= 1000;
	}
	if (func != 4 && systick_1s_status)
	{
		systick_1s_status	= 0;
		
		second++;
		elevation();	
	}
	
}


/*
	Corresponding to the startup_TM4C129.s vector table UART0_Handler interrupt program name
*/

void UART0_Handler(void)
{
	int32_t uart0_int_status;
	
	int cnt2=0;
	char uart_receive_char;
	
  uart0_int_status  =  UARTIntStatus(UART0_BASE, true);		// Get the interrrupt status.

  UARTIntClear(UART0_BASE, uart0_int_status);								//Clear the asserted interrupts
  
	//UARTStringGetNonBlocking();
	
	while( UARTCharsAvail(UART0_BASE) ) 
	{ 
     uart_receive_char= UARTCharGetNonBlocking(UART0_BASE);
		 if(uart_receive_char == ' ' )
		{
			
		}
		else{
			RxBuf[cnt2] = toupper(uart_receive_char);
			cnt2++;
		}
		 		
  }
	
  //RxBuf[cnt]='\0';
	
	
	sprintf(time_record,"%4s%02u%s%02u%s%02u","TIME",hour,":",minute,":",second);
	sprintf(date_record,"%4s%02u%s%02u%s%02u","DATE",year,"-",mon,"-",day);
  sprintf(alarm_record,"%4s%02u%s%02u%s%02u","ALARM",alarm_h,":",alarm_m,":",alarm_s); 	
	
  //HELPָ��
	if((strcasecmp(RxBuf,"HELP") == 0)||strncmp(RxBuf, "?", 1)==0)
	{
		UARTStringPut((uint8_t *)"\r\nPlease enter [INIT CLOCK]to initialize the clock.\r\n");
		UARTStringPut((uint8_t *)"\r\nPlease enter [INIT ALARM]to initialize the clock.\r\n");
		UARTStringPut((uint8_t *)"\r\nPlease enter [TIME**:**:**] to set the clock.\r\n");
		UARTStringPut((uint8_t *)"\r\nPlease enter [DATE**:**:**] to set the date.\r\n");
		UARTStringPut((uint8_t *)"\r\nPlease enter [ALARM**:**:**] to set the ALARM.\r\n");
		UARTStringPut((uint8_t *)"\r\nPlease enter [GET TIME] to get the current time.\r\n");
		UARTStringPut((uint8_t *)"\r\nPlease enter [GET DATE] to get the date.\r\n");
		UARTStringPut((uint8_t *)"\r\nPlease enter [GET ALARM] to get the alarm.\r\n");
		UARTStringPut((uint8_t *)"\r\nPlease enter [RUN SWATCH] to run the countdown_clock.\r\n");
		UARTStringPut((uint8_t *)"\r\n function1:show date.\r\n");
		UARTStringPut((uint8_t *)"\r\n function2:change date.\r\n");
		UARTStringPut((uint8_t *)"\r\n function3:show time.\r\n");
		UARTStringPut((uint8_t *)"\r\n function4:change time.\r\n");
		UARTStringPut((uint8_t *)"\r\n function5:Alarm.\r\n");
		UARTStringPut((uint8_t *)"\r\n function6:Countdown.\r\n");
		//
	}
	
  //INIT CLOCKָ���ʼ��ʱ��Ϊ00:00:00��
  else if((strncasecmp(RxBuf,"INITCLOCK",9) == 0))
	{ 
		  func = 3;
			hour = 00;
			minute = 00;
			second = 00;
			sprintf(time_record,"%4s%02u%s%02u%s%02u","TIME",hour,":",minute,":",second); 
			UARTStringPutNonBlocking(time_record);
			sprintf(func_print,"\r\n");
			UARTStringPutNonBlocking(func_print);
		
	}
	
	//INIT ALARMָ�������ӣ�
  else if((strncasecmp(RxBuf,"INITALARM",9) == 0))
	{ 
		  alarm_enable = 0;   //���ӳ���
		  //�������
			sprintf(time_record,"%13s","ALARM cleared"); 
			UARTStringPutNonBlocking(time_record);
			sprintf(func_print,"\r\n");
			UARTStringPutNonBlocking(func_print);
		
	}
	
	//SET
	//SET TIMEָ��

	else if(strncasecmp(RxBuf,"TIME",4)==0)
	{
		if((RxBuf[6]=='-'&&RxBuf[9]=='-')||(RxBuf[6]==':'&&RxBuf[9]==':'))
		{   
			func = 3;
			hour=(RxBuf[4]-'0')*10+(RxBuf[5]-'0');
			minute=(RxBuf[7]-'0')*10+(RxBuf[8]-'0');
			second=(RxBuf[10]-'0')*10+(RxBuf[11]-'0');
			
			sprintf(time_record,"%4s%02u%s%02u%s%02u","TIME",hour,":",minute,":",second); 
			UARTStringPutNonBlocking(time_record);
			sprintf(func_print,"\r\n");
			UARTStringPutNonBlocking(func_print);
		}
		else {	
			UARTStringPut((uint8_t *)"\r\nerror!\r\n");
			UARTStringPut((uint8_t *)"\r\nYou can enter [HELP] or ? for the help menu.\r\n");		
		}
	}
	
	
	//SET DATEָ��
  else if(strncasecmp(RxBuf,"DATE",4)==0)
	{
		if((RxBuf[8]=='-'&&RxBuf[11]=='-')||(RxBuf[8]==':'&&RxBuf[11]==':'))
		{
			func = 1;
			year1=(RxBuf[4]-'0')*10+(RxBuf[5]-'0');
			year=(RxBuf[6]-'0')*10+(RxBuf[7]-'0');
			mon=(RxBuf[9]-'0')*10+(RxBuf[10]-'0');
			day=(RxBuf[12]-'0')*10+(RxBuf[13]-'0');
			
			sprintf(date_record,"%4s%02u%02u%s%02u%s%02u","DATE",year1,year,"-",mon,"-",day); 
			UARTStringPutNonBlocking(date_record);
			sprintf(func_print,"\r\n");
			UARTStringPutNonBlocking(func_print);
		}
		else {	
			UARTStringPut((uint8_t *)"\r\nerror!\r\n");
			UARTStringPut((uint8_t *)"\r\nYou can enter [HELP] or ? for the help menu.\r\n");		
		}
	}
	
	//SET ALARMָ��
  else if(strncasecmp(RxBuf,"ALARM",5)==0)
	{
		if((RxBuf[7]=='-'&&RxBuf[10]=='-')||(RxBuf[7]==':'&&RxBuf[10]==':'))
		{   
			func = 5;
			alarm_h=(RxBuf[5]-'0')*10+(RxBuf[6]-'0');
			alarm_m=(RxBuf[8]-'0')*10+(RxBuf[9]-'0');
			alarm_s=(RxBuf[11]-'0')*10+(RxBuf[12]-'0');
			
			sprintf(alarm_record,"%4s%02u%s%02u%s%02u","ALARM",alarm_h,":",alarm_m,":",alarm_s); 
			UARTStringPutNonBlocking(alarm_record);
			sprintf(func_print,"\r\n");
		UARTStringPutNonBlocking(func_print);
		}
		else {	
			UARTStringPut((uint8_t *)"\r\nerror!\r\n");
			UARTStringPut((uint8_t *)"\r\nYou can enter [HELP] or ? for the help menu.\r\n");		
		}
	}

	
	//GET TIMEָ��
  else if((strncasecmp(RxBuf,"GETTIME",7) == 0))
	{
		func = 3;
		sprintf(time_record,"%4s%02u%s%02u%s%02u","TIME",hour,":",minute,":",second);
		UARTStringPutNonBlocking(time_record);
		sprintf(func_print,"\r\n");
		UARTStringPutNonBlocking(func_print);
	}
	
	//GET DATEָ��
  else if((strncasecmp(RxBuf,"GETDATE",7) == 0))
	{
		func = 1;
		sprintf(date_record,"%4s%02u%02u%s%02u%s%02u","DATE",year1,year,"-",mon,"-",day);   
		UARTStringPutNonBlocking(date_record);
		sprintf(func_print,"\r\n");
		UARTStringPutNonBlocking(func_print);
	}
	
	//GET ALARMָ��
  else if((strncasecmp(RxBuf,"GETALARM",8) == 0))
	{
		func = 5;
		sprintf(alarm_record,"%4s%02u%s%02u%s%02u","ALARM",alarm_h,":",alarm_m,":",alarm_s);
		UARTStringPutNonBlocking(alarm_record);
		sprintf(func_print,"\r\n");
		UARTStringPutNonBlocking(func_print);
	}
	
	//RUN SWATCHָ��
	else if((strncasecmp(RxBuf,"RUNSWATCH",9) == 0))
	{
		func = 6;
		sprintf(CountDown_record,"%2s%02u%s%02u","CD",cd_s,":",cd_ms);
		UARTStringPutNonBlocking(CountDown_record);
		cd_enable = cd_enable%2+1;   //����ʱʹ��
		sprintf(func_print,"\r\n");
		UARTStringPutNonBlocking(func_print);
	}
	
	//��������ʾ
	else{
		UARTStringPut((uint8_t *)"\r\nerror!\r\n");
		UARTStringPut((uint8_t *)"\r\nYou can enter [HELP] or ? for the help menu.\r\n");
	}
}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               

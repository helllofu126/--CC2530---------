#include <ioCC2530.h>
#include "adc.h"
#include <hal_lcd.h>
#include "hal_led.h"
#include "hal_joystick.h"
#include "hal_assert.h"
#include "hal_board.h"
#include "hal_int.h"
#include "hal_mcu.h"
#include "hal_button.h"
#include "hal_rf.h"
#include "util_lcd.h"
#include "basic_rf.h"
/*************************************************************/
typedef unsigned char uchar;
typedef unsigned int  uint;

//Application paramters
#define RF_CHANNLE 25

//BasicRF address(client)
#define PAN_ID 0x2007
#define CLIENT_ADDR 0X0002
#define SERVER_ADDR 0xBEEF

//Application states
#define IDLE 0
#define SEND_CMD 1

//Application role
#define NONE 0
#define CLIENT 1
#define SERVER 2
#define APP_MODES 2

//Test 
#define LED1 P1_0       //SignalLight
#define LED2 P1_1       //SignalLight
#define DATA_PIN P0_6   //Illumination intensity pin

//**********************Funtion Declarations********************//
void InitLed(void);
uint16 readV(uint8 channal,uint8 resolution);
void initUARTSend(void);
void UART_Send(char *Data,int len);
void sendIlluVal();
uint8 checkSum(char *b,uint8 sumPos);//Returns checksum.
void initRelay();
void relayOn();
void relayOff();
void sendRelayStatus();
uint16 getRelayStatus();
void DelayMS(uint msec)
{ 
  uint i,j,k;
 	
	for(int l=0;l<1000;l++)
		for (i=0; i<msec; i++)
		{
				for (j=0; j<53500;j++)
				{
					for(k=0;k<53500;k++);
				}
		}
}

//**************************************************************//

//Buffers
#define DATA_LEN 5
#define DATA_SEND_LEN 6
#define ILLUM_HEAD 0xff
#define CMD_HEAD 0xff
static uint8 send_buff[DATA_SEND_LEN];
static uint8 rev_buff[DATA_LEN];
static basicRfCfg_t basicRfConfig;
//Parameters for getting illumination intensity.
static uint16 illum_val,Did;
void main(void)
{
	Did=(uint16)CLIENT_ADDR;
  InitLed();
	basicRfConfig.myAddr=CLIENT_ADDR;
  basicRfConfig.panId=PAN_ID;
  basicRfConfig.channel=RF_CHANNLE;
  basicRfConfig.ackRequest=TRUE;
  halBoardInit();
  //initUARTSend();
	
  if(halRfInit()==FAILED) {
  HAL_ASSERT(FALSE);
  }
	halLedSet(1);
	
	basicRfInit(&basicRfConfig);
	basicRfReceiveOn();
	LED1=1;
	initRelay();
	while(1)
	{
		
		/*Checking if there is command from the top device and actting responding action.*/
		if(basicRfPacketIsReady())
		{
			basicRfReceive(rev_buff,DATA_LEN,NULL);
			
			if(rev_buff[0]==0xff&&rev_buff[2]==(uint8)(Did>>8)&&rev_buff[3]==(uint8)Did&&rev_buff[4]==checkSum((char*)rev_buff,4))
			{
				if(rev_buff[1]==0x00)//data requset
				{
					sendIlluVal();
				}
				else if(rev_buff[1]==0xAA)//off cmd
				{
					LED1=1;//And this part.
					relayOff();
				}
				else if(rev_buff[1]==0xff)//on cmd
				{
					LED1=0;//Replace this part.
					relayOn();
				}
				else if(rev_buff[1]==0x02)
				{
					//Retrun the status of relay
					sendRelayStatus();
				}
			}else{};	
		}else{};
		DelayMS(300);
	}
	
}


void InitLed(void)
{
  P1DIR |= 0x01; //1 output
  P0DIR &= ~0x40;//0 input.
  LED1=0;
}

void initRelay()
{
	P0DIR|=0x80;
	P0_7=1;
}

void relayOff()
{
	P0_7=1;
}
void relayOn()
{
	P0_7=0;
}

uint16 getRelayStatus()
{
	return P0_7==1?0xffff:0x0000;
}

void sendRelayStatus()
{
	uint16 s=getRelayStatus();
	send_buff[0]=0xff;
	send_buff[1]=(uint8)(s>>8);
	send_buff[2]=(uint8)s;
	send_buff[3]=(uint8)(Did>>8);
	send_buff[4]=(uint8)Did;
	send_buff[5]=checkSum((char*)send_buff,5);
	
	basicRfSendPacket(SERVER_ADDR, send_buff, DATA_SEND_LEN);
	DelayMS(300);
}

//ADC
uint16 readV(uint8 channal,uint8 resolution)
{
    uint16 value ;
		ADCIF=0;
    APCFG |= 1 << channal ; 
    ADC_ENABLE_CHANNEL(channal);
  
    ADC_SINGLE_CONVERSION(ADC_REF_AVDD | resolution | channal);
    ADC_SAMPLE_SINGLE();
    
//		while (0==(ADCCON1 & 0x80));
		while(!ADCIF);
      
    value = ADCL ;
    value |= ((uint16) ADCH) << 8 ;


    return value;
}

void initUARTSend(void)
{
  
	CLKCONCMD &= ~0x40;
	while(CLKCONSTA & 0x40);
	CLKCONCMD &= ~0x47;
	
	
	PERCFG = 0x00;
	P0SEL = 0x3c;				
	P2DIR &= ~0XC0;            
	U0CSR |= 0x80;
	U0GCR |= 11;				       
	U0BAUD |= 216;			
	UTX0IF = 0;                   
}

void UART_Send(char *Data,int pos)
{
  int j;
  for(j=0;j<pos;j++)
  {
    U0DBUF = *Data++;
    while(UTX0IF == 0);
    UTX0IF = 0;
  }
}

//Sends illumination intensity value
void sendIlluVal()
{
  for(int i=0;i<DATA_LEN;i++)send_buff[i]=0;
  send_buff[0]=0x00;
  //Gets illumination intensity value
  illum_val=readV(ADC_AIN6,ADC_9_BIT);
  //Fills the buffer
  send_buff[1]|=(illum_val>>8);
  send_buff[2]=illum_val;
  send_buff[3]=(uint8)(Did>>8);
	send_buff[4]=(uint8)Did;
	send_buff[5]=checkSum((char*)send_buff,5);
	
  basicRfSendPacket(SERVER_ADDR, send_buff, DATA_SEND_LEN);
	DelayMS(300);
}

uint8 checkSum(char *b,uint8 checkSumPos)
{
  char sum=0;
  for(uint8 i=0;i<checkSumPos;i++)
	sum+=b[i];
  
  return sum;
}

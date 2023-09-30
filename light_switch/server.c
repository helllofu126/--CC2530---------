/****************************************************************************
* 文 件 名: main.c
* 描    述: 光敏实验，有光时LED1亮，用手挡住光敏电阻时LED1熄灭
****************************************************************************/
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
#define CLIENT_ADDR 0XBEE1
#define SERVER_ADDR 0xBEEF


#define LED1 P1_0       //SignalLight

//**********************Funtion Declarations********************//
void InitLed(void);
void initUARTSend(void);
void UARTSend(char *Data,int len);
uint8 checkSum(char *b,uint8 len);//Returns checksum.
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
#define DATA_RCV_LEN 6
uint8 s_buff_pos=0;
static uint8 rev_buff[DATA_RCV_LEN];
static uint8 send_buff[DATA_LEN];
static basicRfCfg_t basicRfConfig;

#define CMD_HEAD 1;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!It should be 1;
#define ILLU_HEAD 0;

//Parameters for getting illumination intensity.

void main(void)
{

	basicRfConfig.myAddr=SERVER_ADDR;
  basicRfConfig.panId=PAN_ID;
  basicRfConfig.channel=RF_CHANNLE;
  basicRfConfig.ackRequest=TRUE;
  halBoardInit();
  initUARTSend();
	
  if(halRfInit()==FAILED) {
  HAL_ASSERT(FALSE);
  }
	halLedSet(1);
	
	basicRfInit(&basicRfConfig);
	basicRfReceiveOn();
	uint16 endDevId;
	InitLed();
	while(1)
	{
		if(s_buff_pos==DATA_LEN)//If the buffer was filled with the data from pc.
		{
			if(send_buff[0]==0xff&&send_buff[4]==checkSum((char*)send_buff,4))
			{
				//Determain which enddevice to send to.
				endDevId=0x00;
				endDevId|=send_buff[2];
				endDevId<<=8;
				endDevId|=send_buff[3];
				LED1=0;
				basicRfSendPacket(endDevId,send_buff,DATA_LEN);
				s_buff_pos=0;
				DelayMS(300);
				LED1=1;
			}else{};
		}else{};
		
		//Checks if there are data from sensor board.
		if(basicRfPacketIsReady())
		{
			if(basicRfReceive(rev_buff,DATA_RCV_LEN,NULL)==6)
			{
				UARTSend((char*)rev_buff,DATA_RCV_LEN);
			}
			else{};
		}else{};
	}
}

void InitLed(void)
{
  //初始化灯的IO口
  P1DIR |= 0x01; 
	LED1=1;
}


//串口初始化与发送函数
void initUARTSend(void)
{
 
	CLKCONCMD &= ~0x40;
	while(CLKCONSTA & 0x40);
	CLKCONCMD &= ~0x47;
	
	PERCFG = 0x00;
	P0SEL  = 0x3c;
	P2DIR &= ~0XC0;            
	U0CSR |= 0x80;
	U0GCR |= 11;				       
	U0BAUD |= 216;			
	UTX0IF = 0;
	
	U0CSR |= 0X40; //允许接收
	IEN0 |= 0x84; //开总中断，接收中断
}

void UARTSend(char *Data,int len)
{
  int j;
  for(j=0;j<len;j++)
  {
    U0DBUF = *Data++;
    while(UTX0IF == 0);
    UTX0IF = 0;
  }
}

uint8 checkSum(char *b,uint8 len)
{
  char sum=0;
  for(uint8 i=0;i<len;i++)
	sum+=b[i];
  
  return sum;
}

#pragma vector = URX0_VECTOR
__interrupt void UART0_ISR(void)
{
	
	if(s_buff_pos==DATA_LEN)//If sending buffer is full,do not fill.
		return;
	URX0IF=0;
	//Fills the sending buffer.
	send_buff[s_buff_pos++]=U0DBUF;
}

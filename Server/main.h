#include "reg52.h"

typedef unsigned char	uint8;
typedef unsigned int	uint16;
typedef unsigned long	uint32;


//sbit	bRunLED			=	P3^4;	//运行指示灯
//sbit	bWatchDog		=	P1^7;	//看门狗复位
//sbit	b485Send		= 	P1^3;	//75LBC184 发送接收控制
sbit	b485Send		= 	P1^2;	//75LBC184 发送接收控制
sbit	bRunLED			=	P1^1;	//运行指示灯
//sbit    bRecvFlag = P1^5   ; //接受到OK的数据

//sbit	bSendLED			=	P1^4;	//运行指示灯

sbit 	bSwitchLed 			=   P1^5;
#define TIMER_HIGHT	0xf8
#define TIMER_LOW	0xcd
 
sbit	bClient			=	P2^0;	//客户端1
sbit	bClient2		=	P2^1;	//客户端2
sbit	bClient3		=	P2^2;	//客户端3
sbit	bClient4		=	P2^3;	//客户端4
sbit	bClient5		=	P2^4;	//客户端5
sbit	bClient6		=	P2^5;	//客户端6
sbit	bClient7		=	P2^6;	//客户端7
sbit	bClient8		=	P2^7;	//客户端7

sfr WDT_CONTR=0xe1;

extern uint8	idata sendBuf[16],receBuf[16];
extern uint8	idata checkoutError;	// ==2 偶校验错 
extern uint8	idata receTimeOut;
extern uint32 	dwTickCount;


#include "modbus.h"


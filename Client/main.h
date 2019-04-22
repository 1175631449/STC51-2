#include "reg52.h"
#include <intrins.h> //Keil library (is used for _nop()_ operation)  
#include <math.h>    //Keil library  
#include <stdio.h>   //Keil library

typedef unsigned char	uint8;
typedef unsigned int	uint16;
typedef unsigned long	uint32;

sbit	bServer			=	P1^0;	//服务器
//sbit	bRunLED			=	P3^4;	//运行指示灯
//sbit	bWatchDog		=	P1^7;	//看门狗复位
//sbit	b485Send		= 	P1^3;	//75LBC184 发送接收控制
sbit	b485Send		= 	P1^2;	//75LBC184 发送接收控制
sbit	bRunLED			=	P1^4;	//运行指示灯
//sbit    bRecvFlag = P1^5   ; //接受到OK的数据
//sbit	bRecvLED			=	P1^3;	//运行指示灯
sbit	bSendLED			=	P1^5;	//运行指示灯
//sbit	bSignLED			=	P1^7;	//运行指示灯
#define TIMER_HIGHT	0xf8
#define TIMER_LOW	0xcd
 

sfr WDT_CONTR=0xe1;

extern uint8	idata sendBuf[16],receBuf[16];
extern uint8	idata checkoutError;	// ==2 偶校验错 
extern uint8	idata receTimeOut;
extern uint32 	dwTickCount;


#include "modbus.h"


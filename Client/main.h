#include "reg52.h"
#include <intrins.h> //Keil library (is used for _nop()_ operation)  
#include <math.h>    //Keil library  
#include <stdio.h>   //Keil library

typedef unsigned char	uint8;
typedef unsigned int	uint16;
typedef unsigned long	uint32;

sbit	bServer			=	P1^0;	//������
//sbit	bRunLED			=	P3^4;	//����ָʾ��
//sbit	bWatchDog		=	P1^7;	//���Ź���λ
//sbit	b485Send		= 	P1^3;	//75LBC184 ���ͽ��տ���
sbit	b485Send		= 	P1^2;	//75LBC184 ���ͽ��տ���
sbit	bRunLED			=	P1^4;	//����ָʾ��
//sbit    bRecvFlag = P1^5   ; //���ܵ�OK������
//sbit	bRecvLED			=	P1^3;	//����ָʾ��
sbit	bSendLED			=	P1^5;	//����ָʾ��
//sbit	bSignLED			=	P1^7;	//����ָʾ��
#define TIMER_HIGHT	0xf8
#define TIMER_LOW	0xcd
 

sfr WDT_CONTR=0xe1;

extern uint8	idata sendBuf[16],receBuf[16];
extern uint8	idata checkoutError;	// ==2 żУ��� 
extern uint8	idata receTimeOut;
extern uint32 	dwTickCount;


#include "modbus.h"


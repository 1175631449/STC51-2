#include "reg52.h"

typedef unsigned char	uint8;
typedef unsigned int	uint16;
typedef unsigned long	uint32;


//sbit	bRunLED			=	P3^4;	//����ָʾ��
//sbit	bWatchDog		=	P1^7;	//���Ź���λ
//sbit	b485Send		= 	P1^3;	//75LBC184 ���ͽ��տ���
sbit	b485Send		= 	P1^2;	//75LBC184 ���ͽ��տ���
sbit	bRunLED			=	P1^1;	//����ָʾ��
//sbit    bRecvFlag = P1^5   ; //���ܵ�OK������

//sbit	bSendLED			=	P1^4;	//����ָʾ��

sbit 	bSwitchLed 			=   P1^5;
#define TIMER_HIGHT	0xf8
#define TIMER_LOW	0xcd
 
sbit	bClient			=	P2^0;	//�ͻ���1
sbit	bClient2		=	P2^1;	//�ͻ���2
sbit	bClient3		=	P2^2;	//�ͻ���3
sbit	bClient4		=	P2^3;	//�ͻ���4
sbit	bClient5		=	P2^4;	//�ͻ���5
sbit	bClient6		=	P2^5;	//�ͻ���6
sbit	bClient7		=	P2^6;	//�ͻ���7
sbit	bClient8		=	P2^7;	//�ͻ���7

sfr WDT_CONTR=0xe1;

extern uint8	idata sendBuf[16],receBuf[16];
extern uint8	idata checkoutError;	// ==2 żУ��� 
extern uint8	idata receTimeOut;
extern uint32 	dwTickCount;


#include "modbus.h"


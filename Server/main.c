#include "main.h"

/******************************
微控电子 www.mcuc.cn
modbus RTU 的C51程序 
单片机89S52
通信波特率 9600 8位数据 1位停止位 偶校验 485通位接口
单片机控制板地址 localAddr(变量)
通信可设置数据的地址：
字地址 0 - 255 (只取16位的低8位)
位地址 0 - 255 (只取16位的低8位)
*******************************/

uint32	dwTickCount,dwIntTick;	//时钟
uint8	idata sendBuf[16],receBuf[16]; //发送接收缓冲区
uint8	idata checkoutError;	// ==2 偶校验错  
uint8	idata receTimeOut;		//接收超时
uint8	idata c10ms;			//10ms 计时
bit		b1ms,bt1ms,b10ms,bt10ms,b100ms,bt100ms;	//定时标志位
uint8   idata clientErrors[16];




#define SERVER_ADDR 0
#define ACTION_GET 0x1	   //获取状态

//#define CLIENT_ADDR_START  7
//#define CLIENT_ADDR_END 8

//一期
//const uint8 clientids[]={1,2,3,4,5,6,10,12};
//const uint8 CLIENTCOUNT = 8;

//二期
//const uint8  clientids[]={30,31,32  };

const uint8  clientids[]={40,41 };


const uint8 CLIENTCOUNT = 2;


uint8 curClientAddr ; 

uint8 lastRecvFlag;

uint8 h232Mode;

uint8 nRetryCount;


#define MAX_RETRYCOUNT 8



																		
// 串行中断程序
void commIntProc() interrupt 4
{
	if(TI)
		{
			TI = 0; 
			if(sendPosi < sendCount) 
			{
				sendPosi++;
				ACC = sendBuf[sendPosi];
				TB8 = P;	//加上校验位
				SBUF = sendBuf[sendPosi];
//				bSendLED = !bSendLED;
			}
			else
			{
				SetSendFlag(0);			   //发送完后将485置于接收状态
				receCount = 0;   //清接收地址偏移寄存器
				checkoutError = 0;
//				bSendLED = 1;
			}
		}
		else if(RI)
		{
			RI = 0;
			receTimeOut = 10;    //通讯超时值
			receBuf[receCount] = SBUF;
			ACC = receBuf[receCount];
			if(P != RB8)
				checkoutError = 2;	//偶校验出错
			receCount++;          //接收地址偏移寄存器加1
			receCount &= 0x0f;    //最多一次只能接收16个字节
		//	bSendLED = !bSendLED;
		//	if(	 receCount == 16)
		//		 bSendLED = 0;
//			bSendLED = 0;
		}



}   // void CommIntProc()

//定时器0 1ms 中断
void timer0IntProc() interrupt 1
{
	TL0 = TIMER_LOW; 
    TH0 = TIMER_HIGHT;
    dwIntTick++;
	bt1ms = 1;
    c10ms++;
    if(c10ms >= 10) 
    {
        c10ms = 0;      //10ms计时器清零
        bt10ms = 1;
    }


	 if(c10ms %3 == 0 )
   {
   		WDT_CONTR=0x35; 
		
   }

}   // void Timer0IntProc()

//外部中断0
void intEx0Proc(void) interrupt 0
{

}

//计数器1中断
void counter1IntProc(void) interrupt 3 using 1
{

}


void SetClientState(uint8 client,uint8 err,uint8 state,uint16 fHumi,uint16 fTempl)
{
	uint8 i,index;
	uint8 * pH = (uint8*)&	  fHumi;
	uint8 * pT = (uint8*)&	  fTempl;

//	bSignLED = 	 !bSignLED;

	index = client%8;

//	if(client>16) 
//		return;

	//获取客户端的错误存储位置
	if(err == 0)
	{
		//没有错误
		clientErrors[index] = 0;
	}
	else
	{
		clientErrors[index] ++;
		if(clientErrors[index]>=3)
		{
			clientErrors[index] = 0;	
		}
		else
		{
			//不登记此次错误
			return;
		}	
	}	


	


	switch(	index) //20130927 调整  ,原来的代码 client%8 会把8丢掉 
	{
		case 1:
			bClient = (state == 1 &&  err == 0)?0:1;
			break;
		case 2:
			bClient2 = (state == 1 &&  err == 0)?0:1;
			break;
		case 3:
			bClient3 = (state == 1 &&  err == 0)?0:1;
			break;
		case 4:
			bClient4 = (state == 1 &&  err == 0)?0:1;
			break;
		case 5:
			bClient5 = (state == 1 &&  err == 0)?0:1;
			break;
		case 6:
			bClient6 = (state == 1 &&  err == 0)?0:1;
			break;
		case 7:
			bClient7 = (state == 1 &&  err == 0)?0:1;
			break;
		case 0:
			bClient8 = (state == 1 &&  err == 0)?0:1;
			break;
			


	/*			case CLIENT_ADDR_START:
			bClient7 = (state == 1 &&  err == 0)?0:1;
			break;
		case CLIENT_ADDR_START+1:
			bClient8 = (state == 1 &&  err == 0)?0:1;
			break;


		case CLIENT_ADDR_START:
			bClient = (state == 1 &&  err == 0)?0:1;
			break;
		case CLIENT_ADDR_START+1:
			bClient2 = (state == 1 &&  err == 0)?0:1;
			break;
		case CLIENT_ADDR_START+2:
			bClient3 = (state == 1 &&  err == 0)?0:1;
			break;
		case CLIENT_ADDR_START+3:
			bClient4 = (state == 1 &&  err == 0)?0:1;
			break;
		case CLIENT_ADDR_START+4:
			bClient5 = (state == 1 &&  err == 0)?0:1;
			break;
		case CLIENT_ADDR_START+5:
			bClient6 = (state == 1 &&  err == 0)?0:1;
			break;
		case CLIENT_ADDR_START+6:
			bClient7 = (state == 1 &&  err == 0)?0:1;
			break;
		case CLIENT_ADDR_START+7:
			bClient8 = (state == 1 &&  err == 0)?0:1;
			break;
			*/
		default:
			break;
	}

	h232Mode = 1;
	bSwitchLed = 1;

   	sendBuf[0] = 'A';
	sendBuf[1] = 'B';
	sendBuf[2] = 'C';
	 
	sendBuf[3] =  client;
	sendBuf[4] =  err;
	
	sendBuf[5] =  state;

	sendBuf[6] =  (uint8)( fHumi/1000) ;
	sendBuf[7] = (uint8)( fHumi%1000/100) ;
	sendBuf[8] = (uint8)( fHumi%100/10) ;
	sendBuf[9] =  (uint8)( fHumi%10) ;


	
	sendBuf[10] =  (uint8)( fTempl/1000) ;
	sendBuf[11] =  (uint8)( fTempl%1000/100) ;
	sendBuf[12] =  (uint8)( fTempl%100/10) ;
	sendBuf[13] = (uint8)( fTempl%10) ;

	sendBuf[14] = 'D';
	sendBuf[15] = 'E';

	//暂停串口中断
	ES = 0;

  	for( i=0;i<16;i++ )
	{
		ACC = sendBuf[i];
		TB8 = P;	//加上校验位
		SBUF = sendBuf[i];
		while(TI==0);
		TI=0;
	}

	//打开串口中断
	ES = 1;
	bSwitchLed = 0;
}


//初始化串口
void initUart(void)
{
	h232Mode = 0;
	bSwitchLed =0;
	//T2 用于波特率 9600
	T2CON = 0x30;
	RCAP2H = 0xff;
	RCAP2L = 0xb8;
	TR2 = 1;

	//偶校验 	
	//串口工作方式3，11UART					
	SCON = 0xd0;
    PCON = 0;
    ES = 1;


}//void initUart(void)

//定时处理
void timeProc(void)
{
	static uint8 c200ms;

   // bWatchDog = ~ bWatchDog;    //看门狗取反
	b1ms = 0;
	b10ms = 0;
	b100ms = 0;
	
	ET0 = 0;
	dwTickCount = dwIntTick;
	ET0 = 1;

	if(bt1ms)
	{
		bt1ms = 0;
		b1ms = 1;

        if(receTimeOut>0)
        {
            receTimeOut--;
            if(receTimeOut==0 && receCount>0)   //判断通讯接收是否超时
            {
 				SetSendFlag(0); //将485置为接收状态
			    receCount = 0;      //将接收地址偏移寄存器清零
				checkoutError = 0;
            }
        }
	}
	
	if(bt100ms)
	{
		bt100ms = 0;
		b100ms = 1;
	}
    if(bt10ms)      //判断中断10ms标志位是否1
    {
        bt10ms = 0;     //清中断10ms标志位
		b10ms = 1;

        c200ms++;                   //200ms计时器加1
        if(c200ms >= 200)            //判断是否计时到2000ms
        {
			bRunLED = !bRunLED;
            c200ms = 0;             //清200ms计时器
			
        }


		if(c200ms % 8 == 0 && (bSwitchLed == 0))  //100毫秒做一次
		{
			if(curClientAddr!= 0 && lastRecvFlag == 0)
			{
				//上次发出的包没有收到
				nRetryCount ++ ;
				if(	  nRetryCount >=   MAX_RETRYCOUNT)
				{
					//超过最大的重试次数
					SetClientState(	clientids[curClientAddr-1],1,0,0,0);
					lastRecvFlag = 1;

					//检查下一个节点
					curClientAddr ++;
					nRetryCount = 0; //重新开始
				}
			}
			else
			{
				if(curClientAddr!=0)
					curClientAddr ++;
				nRetryCount = 0; //重新开始
			}
		
	
				if(curClientAddr == 0)
					curClientAddr =  1;
				if(	 curClientAddr >  CLIENTCOUNT  )
					curClientAddr =  1;

		    	sendBuf[0] =  clientids[curClientAddr-1];
				sendBuf[1] =  ACTION_GET;
				bSwitchLed = 0;
				h232Mode =0;
				lastRecvFlag = 0;
				sendCount = 2;
				beginSend();
		}	 
    }
}   // void TimerProc(void)


//初始化中断
void initInt(void)
{
	/*T1 为计数器 T0 为计时器 ，使用工作方式2，使用TH与TL作为寄存器*/
	TMOD = 0x51;

	//初始化定时器 0 开始值
	TH0 = TIMER_HIGHT;
	TL0 = TIMER_LOW;
	TR0 = 1;	
    ET0 = 1;
	TH1 = 0;			//9600
    TL1 = 0;
	TR1 = 0;			//定时器1用于计数定时器2用于波特

	ET1 = 1;
	//PT1 = 1;

	IT0 = 1;	
    IT1 = 1;
	EX0 = 0;	
	PX0 = 1;
    EX1 = 0;

	initUart();

	//打开全部中断
	EA = 1;		
}   // void initInt(void)

//初始化
void initProg(void)
{	
	initInt();
	SetSendFlag(0); //将485置为接收状态
}

void main(void)
{
	 uint8 i;

	initProg();

	bClient = 1;
	bClient2 = 1;
	bClient3 = 1;
	bClient4 = 1;
	bClient5 = 1;
	bClient6 = 1;
	bClient7 = 1;
	bClient8 = 1;
								

	for(i = 0;i<16;i++)
	{
		clientErrors[i] = 0;
	}


	//喂狗
	WDT_CONTR=0x35; 


	bRunLED  = 0;
	for(  i = 0;i<16;i++)
	{
		sendBuf[i]= 0;	
	}
	receCount = 0;
//	bSendLED = 1;

	SetSendFlag(0); //将485置为接收状态
	curClientAddr = 0;
	lastRecvFlag = 0;
	h232Mode = 0;
	bSwitchLed =0;
	nRetryCount = 0;

	while(1)
	{
		timeProc();
		if(	receCount == 12)
		{
			if(	receBuf[0] == 	   SERVER_ADDR && checkoutError == 0)
			{
				if(	(curClientAddr!=0)&& (receBuf[1] == 	clientids[curClientAddr-1]))
				{
					lastRecvFlag = 1;
					switch(	 receBuf[2])
					{
						case ACTION_GET:
							 {
								SetClientState(	clientids[curClientAddr-1],0,receBuf[3],
									 receBuf[4]*1000+	   receBuf[5]*100+receBuf[6]*10+receBuf[7],
									  receBuf[8]*1000+	   receBuf[9]*100+receBuf[10]*10+receBuf[11]);
								
							 }
							break;
						default:
							break;
					}

				}
				
			}
			receCount = 0;
			checkoutError = 0;
		}
	}
}

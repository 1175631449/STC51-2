#include "main.h"

/******************************
΢�ص��� www.mcuc.cn
modbus RTU ��C51���� 
��Ƭ��89S52
ͨ�Ų����� 9600 8λ���� 1λֹͣλ żУ�� 485ͨλ�ӿ�
��Ƭ�����ư��ַ localAddr(����)
ͨ�ſ��������ݵĵ�ַ��
�ֵ�ַ 0 - 255 (ֻȡ16λ�ĵ�8λ)
λ��ַ 0 - 255 (ֻȡ16λ�ĵ�8λ)
*******************************/

uint32	dwTickCount,dwIntTick;	//ʱ��
uint8	idata sendBuf[16],receBuf[16]; //���ͽ��ջ�����
uint8	idata checkoutError;	// ==2 żУ���  
uint8	idata receTimeOut;		//���ճ�ʱ
uint8	idata c10ms;			//10ms ��ʱ
bit		b1ms,bt1ms,b10ms,bt10ms,b100ms,bt100ms;	//��ʱ��־λ
uint8   idata clientErrors[16];




#define SERVER_ADDR 0
#define ACTION_GET 0x1	   //��ȡ״̬

//#define CLIENT_ADDR_START  7
//#define CLIENT_ADDR_END 8

//һ��
//const uint8 clientids[]={1,2,3,4,5,6,10,12};
//const uint8 CLIENTCOUNT = 8;

//����
//const uint8  clientids[]={30,31,32  };

const uint8  clientids[]={40,41 };


const uint8 CLIENTCOUNT = 2;


uint8 curClientAddr ; 

uint8 lastRecvFlag;

uint8 h232Mode;

uint8 nRetryCount;


#define MAX_RETRYCOUNT 8



																		
// �����жϳ���
void commIntProc() interrupt 4
{
	if(TI)
		{
			TI = 0; 
			if(sendPosi < sendCount) 
			{
				sendPosi++;
				ACC = sendBuf[sendPosi];
				TB8 = P;	//����У��λ
				SBUF = sendBuf[sendPosi];
//				bSendLED = !bSendLED;
			}
			else
			{
				SetSendFlag(0);			   //�������485���ڽ���״̬
				receCount = 0;   //����յ�ַƫ�ƼĴ���
				checkoutError = 0;
//				bSendLED = 1;
			}
		}
		else if(RI)
		{
			RI = 0;
			receTimeOut = 10;    //ͨѶ��ʱֵ
			receBuf[receCount] = SBUF;
			ACC = receBuf[receCount];
			if(P != RB8)
				checkoutError = 2;	//żУ�����
			receCount++;          //���յ�ַƫ�ƼĴ�����1
			receCount &= 0x0f;    //���һ��ֻ�ܽ���16���ֽ�
		//	bSendLED = !bSendLED;
		//	if(	 receCount == 16)
		//		 bSendLED = 0;
//			bSendLED = 0;
		}



}   // void CommIntProc()

//��ʱ��0 1ms �ж�
void timer0IntProc() interrupt 1
{
	TL0 = TIMER_LOW; 
    TH0 = TIMER_HIGHT;
    dwIntTick++;
	bt1ms = 1;
    c10ms++;
    if(c10ms >= 10) 
    {
        c10ms = 0;      //10ms��ʱ������
        bt10ms = 1;
    }


	 if(c10ms %3 == 0 )
   {
   		WDT_CONTR=0x35; 
		
   }

}   // void Timer0IntProc()

//�ⲿ�ж�0
void intEx0Proc(void) interrupt 0
{

}

//������1�ж�
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

	//��ȡ�ͻ��˵Ĵ���洢λ��
	if(err == 0)
	{
		//û�д���
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
			//���ǼǴ˴δ���
			return;
		}	
	}	


	


	switch(	index) //20130927 ����  ,ԭ���Ĵ��� client%8 ���8���� 
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

	//��ͣ�����ж�
	ES = 0;

  	for( i=0;i<16;i++ )
	{
		ACC = sendBuf[i];
		TB8 = P;	//����У��λ
		SBUF = sendBuf[i];
		while(TI==0);
		TI=0;
	}

	//�򿪴����ж�
	ES = 1;
	bSwitchLed = 0;
}


//��ʼ������
void initUart(void)
{
	h232Mode = 0;
	bSwitchLed =0;
	//T2 ���ڲ����� 9600
	T2CON = 0x30;
	RCAP2H = 0xff;
	RCAP2L = 0xb8;
	TR2 = 1;

	//żУ�� 	
	//���ڹ�����ʽ3��11UART					
	SCON = 0xd0;
    PCON = 0;
    ES = 1;


}//void initUart(void)

//��ʱ����
void timeProc(void)
{
	static uint8 c200ms;

   // bWatchDog = ~ bWatchDog;    //���Ź�ȡ��
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
            if(receTimeOut==0 && receCount>0)   //�ж�ͨѶ�����Ƿ�ʱ
            {
 				SetSendFlag(0); //��485��Ϊ����״̬
			    receCount = 0;      //�����յ�ַƫ�ƼĴ�������
				checkoutError = 0;
            }
        }
	}
	
	if(bt100ms)
	{
		bt100ms = 0;
		b100ms = 1;
	}
    if(bt10ms)      //�ж��ж�10ms��־λ�Ƿ�1
    {
        bt10ms = 0;     //���ж�10ms��־λ
		b10ms = 1;

        c200ms++;                   //200ms��ʱ����1
        if(c200ms >= 200)            //�ж��Ƿ��ʱ��2000ms
        {
			bRunLED = !bRunLED;
            c200ms = 0;             //��200ms��ʱ��
			
        }


		if(c200ms % 8 == 0 && (bSwitchLed == 0))  //100������һ��
		{
			if(curClientAddr!= 0 && lastRecvFlag == 0)
			{
				//�ϴη����İ�û���յ�
				nRetryCount ++ ;
				if(	  nRetryCount >=   MAX_RETRYCOUNT)
				{
					//�����������Դ���
					SetClientState(	clientids[curClientAddr-1],1,0,0,0);
					lastRecvFlag = 1;

					//�����һ���ڵ�
					curClientAddr ++;
					nRetryCount = 0; //���¿�ʼ
				}
			}
			else
			{
				if(curClientAddr!=0)
					curClientAddr ++;
				nRetryCount = 0; //���¿�ʼ
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


//��ʼ���ж�
void initInt(void)
{
	/*T1 Ϊ������ T0 Ϊ��ʱ�� ��ʹ�ù�����ʽ2��ʹ��TH��TL��Ϊ�Ĵ���*/
	TMOD = 0x51;

	//��ʼ����ʱ�� 0 ��ʼֵ
	TH0 = TIMER_HIGHT;
	TL0 = TIMER_LOW;
	TR0 = 1;	
    ET0 = 1;
	TH1 = 0;			//9600
    TL1 = 0;
	TR1 = 0;			//��ʱ��1���ڼ�����ʱ��2���ڲ���

	ET1 = 1;
	//PT1 = 1;

	IT0 = 1;	
    IT1 = 1;
	EX0 = 0;	
	PX0 = 1;
    EX1 = 0;

	initUart();

	//��ȫ���ж�
	EA = 1;		
}   // void initInt(void)

//��ʼ��
void initProg(void)
{	
	initInt();
	SetSendFlag(0); //��485��Ϊ����״̬
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


	//ι��
	WDT_CONTR=0x35; 


	bRunLED  = 0;
	for(  i = 0;i<16;i++)
	{
		sendBuf[i]= 0;	
	}
	receCount = 0;
//	bSendLED = 1;

	SetSendFlag(0); //��485��Ϊ����״̬
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

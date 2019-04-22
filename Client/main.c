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
uint8   idata ledtick;

uint8 nClientID ;

/* 开始  温度传感器代码 */
typedef union 
{ unsigned int i;
  float f;
 // uint8 arr[4];
} value;

sbit	SCK  = 	P1^0  ;
sbit	DATA = 	P1^1  ;

sbit    SIMSTAT = P1^3;

#define noACK 0
#define ACK   1
                            //adr  command  r/w
#define STATUS_REG_W 0x06   //000   0011    0
#define STATUS_REG_R 0x07   //000   0011    1
#define MEASURE_TEMP 0x03   //000   0001    1
#define MEASURE_HUMI 0x05   //000   0010    1
#define RESET        0x1e   //000   1111    0

enum {TEMP,HUMI};


value humi_val,temp_val;
float dew_point;

/* 结束  温度传感器代码 */


//开始MAX7219 代码
sbit dis_DIN=P2^0; /*显示串行数据输入端*/ 

sbit dis_LOAD=P2^1; /*显示数据锁存端*/ 

sbit dis_CLK=P2^2; /*显示时钟输入端*/ 


sbit c_1 = P2^3;
sbit c_2 = P2^4;
sbit c_3 = P2^5;


#define NoOp 0x00 /*空操作*/ 

#define Digit0 0x01 /*数码管1*/ 

#define Digit1 0x02 /*数码管2*/ 

#define Digit2 0x03 /*数码管3*/ 

#define Digit3 0x04 /*数码管4*/ 

#define Digit4 0x05 /*数码管5*/ 

#define Digit5 0x06 /*数码管6*/ 

#define Digit6 0x07 /*数码管7*/ 

#define Digit7 0x08 /*数码管8*/ 

#define DecodeMode 0x09 /*译码模式*/ 

#define Intensity 0x0a /*亮度*/ 

#define ScanLimit 0x0b /*扫描界限*/ 

#define ShutDown 0x0c /*掉电模式*/ 

#define DisplayTest 0x0f /*显示测试*/ 



#define ShutdownMode 0x00 /*掉电方式工作*/ 

#define NormalOperation 0x01 /*正常操作方式*/ 



#define DecodeDigit 0xff /*译码位数设置*/ 



#define IntensityGrade 0x0a /*显示亮度级别设置*/ 



#define ScanDigit 0x07 /*扫描位数设置*/ 



#define TestMode 0x01 /*显示测试方式*/ 

#define TextEnd 0x00 /*显示测试结束，正常工作*/ 



unsigned char data DisBuffer[8]={0,0,0,0,0,0,0,0}; /*显示缓存区*/ 

//结束MAX7219代码

//

//#define CLIENT_ADDR 0x1
#define ACTION_GET 0x1	   //获取状态


/*-------------------------------------------------------- 

10ms延时子程序 

---------------------------------------------------------*/ 




/*------------------------------------------------- 

向MAX7219写入字节（8位） 

--------------------------------------------------*/ 

void SendChar (unsigned char ch) 
{ 
	unsigned char i,temp; 
	_nop_(); 

	for (i=0;i<8;i++) 
	{ 
			dis_CLK=0;
		temp=ch&0x80; 
		ch=ch<<1; 
		if(temp) 
		{ 
			dis_DIN=1;
		}
		else 
		{ 
			dis_DIN=0;
		} 
		dis_CLK=1;
	} 

} 



/*------------------------------------------------- 

向MAX7219写入字（16位） 

-------------------------------------------------*/ 

void WriteWord (unsigned char addr,unsigned char num) 
{ 
	dis_LOAD=0; 
	_nop_(); 
	SendChar (addr); 
	_nop_(); 
	SendChar (num); 
	_nop_(); 
	dis_LOAD=1; 
} 

/*------------------------------------------------- 

MAX7219初始化 

-------------------------------------------------*/ 
void InitDis (void) 
{ 

  /*	Write_Max7219(SHUT_DOWN, 0x01);   //Normal Operation XXXXXXX1 Shutdown Mode   XXXXXXXX0
 Write_Max7219(DISPLAY_TEST, 0x00);   //Normal Operation XXXXXXX0 Display Test Mode XXXXXXXX1
 Write_Max7219(DECODE_MODE, 0xff);   //Decode Mode Select D7~D0 1 B decode 0 No decode 
 Write_Max7219(SCAN_LIMIT, 0x07);   //SCAN LIMIT 0~7 0xX0~0xX7
 Write_Max7219(INTENSITY, 0x04);   //Set Intensity   0xX0~0xXf	 

	WriteWord (ShutDown,0x01);
	WriteWord (DisplayTest, 0x00); 
	WriteWord (DecodeMode,0xff); 
	WriteWord (ScanLimit,0x07); 
	WriteWord (Intensity, 0x04);	  */

	WriteWord (ScanLimit,ScanDigit); /*设置扫描界限*/ 
	WriteWord (DecodeMode,DecodeDigit); /*设置译码模式*/ 
	WriteWord (Intensity,IntensityGrade); /*设置亮度*/ 
	WriteWord (ShutDown,NormalOperation); /*设置电源工作模式*/ 
   	WriteWord (Intensity, 0x04); /*设置亮度*/ 


} 


//----------------------------------------------------------------------------------
char s_write_byte(unsigned char value)
//----------------------------------------------------------------------------------
// writes a byte on the Sensibus and checks the acknowledge 
{ 
  unsigned char i,error=0;  
  for (i=0x80;i>0;i/=2)             //shift bit for masking
  { if (i & value) DATA=1;          //masking value with i , write to SENSI-BUS
    else DATA=0;                        
    SCK=1;                          //clk for SENSI-BUS
    _nop_();_nop_();_nop_();        //pulswith approx. 5 us  	
    SCK=0;
  }
  DATA=1;                           //release DATA-line
  SCK=1;                            //clk #9 for ack 
  error=DATA;                       //check ack (DATA will be pulled down by SHT11)
  SCK=0;        
  return error;                     //error=1 in case of no acknowledge
}

//----------------------------------------------------------------------------------
char s_read_byte(unsigned char ack)
//----------------------------------------------------------------------------------
// reads a byte form the Sensibus and gives an acknowledge in case of "ack=1" 
{ 
   unsigned char i,val=0;
  DATA=1;                           //release DATA-line
  for (i=0x80;i>0;i/=2)             //shift bit for masking
  { SCK=1;                          //clk for SENSI-BUS
    if (DATA) val=(val | i);        //read bit  
    SCK=0;  					 
  }
  DATA=!ack;                        //in case of "ack==1" pull down DATA-Line
  SCK=1;                            //clk #9 for ack
  _nop_();_nop_();_nop_();          //pulswith approx. 5 us 
  SCK=0;						    
  DATA=1;                           //release DATA-line
  return val;
}

//----------------------------------------------------------------------------------
void s_transstart(void)
//----------------------------------------------------------------------------------
// generates a transmission start 
//       _____         ________
// DATA:      |_______|
//           ___     ___
// SCK : ___|   |___|   |______
{  
   DATA=1; SCK=0;                   //Initial state
   _nop_();
   SCK=1;
   _nop_();
   DATA=0;
   _nop_();
   SCK=0;  
   _nop_();_nop_();_nop_();
   SCK=1;
   _nop_();
   DATA=1;		   
   _nop_();
   SCK=0;		   
}

//----------------------------------------------------------------------------------
void s_connectionreset(void)
//----------------------------------------------------------------------------------
// communication reset: DATA-line=1 and at least 9 SCK cycles followed by transstart
//       _____________________________________________________         ________
// DATA:                                                      |_______|
//          _    _    _    _    _    _    _    _    _        ___     ___
// SCK : __| |__| |__| |__| |__| |__| |__| |__| |__| |______|   |___|   |______
{  
  unsigned char i; 
  DATA=1; SCK=0;                    //Initial state
  for(i=0;i<9;i++)                  //9 SCK cycles
  { SCK=1;
    SCK=0;
  }
  s_transstart();                   //transmission start
}

/*
//----------------------------------------------------------------------------------
char s_softreset(void)
//----------------------------------------------------------------------------------
// resets the sensor by a softreset 
{ 
  unsigned char error=0;  
  s_connectionreset();              //reset communication
  error+=s_write_byte(RESET);       //send RESET-command to sensor
  return error;                     //error=1 in case of no response form the sensor
}	

//----------------------------------------------------------------------------------
char s_read_statusreg(unsigned char *p_value, unsigned char *p_checksum)
//----------------------------------------------------------------------------------
// reads the status register with checksum (8-bit)
{ 
  unsigned char error=0;
  s_transstart();                   //transmission start
  error=s_write_byte(STATUS_REG_R); //send command to sensor
  *p_value=s_read_byte(ACK);        //read status register (8-bit)
  *p_checksum=s_read_byte(noACK);   //read checksum (8-bit)  
  return error;                     //error=1 in case of no response form the sensor
}

//----------------------------------------------------------------------------------
char s_write_statusreg(unsigned char *p_value)
//----------------------------------------------------------------------------------
// writes the status register with checksum (8-bit)
{ 
  unsigned char error=0;
  s_transstart();                   //transmission start
  error+=s_write_byte(STATUS_REG_W);//send command to sensor
  error+=s_write_byte(*p_value);    //send value of status register
  return error;                     //error>=1 in case of no response form the sensor
}	 */
 							   
//----------------------------------------------------------------------------------
char s_measure(unsigned char *p_value, unsigned char *p_checksum, unsigned char mode)
//----------------------------------------------------------------------------------
// makes a measurement (humidity/temperature) with checksum
{ 
  unsigned error=0;
  unsigned int i;

  s_transstart();                   //transmission start
  switch(mode){                     //send command to sensor
    case TEMP	: error+=s_write_byte(MEASURE_TEMP); break;
    case HUMI	: error+=s_write_byte(MEASURE_HUMI); break;
    default     : break;	 
  }
  for (i=0;i<65535;i++) if(DATA==0) break; //wait until sensor has finished the measurement
  if(DATA) error+=1;                // or timeout (~2 sec.) is reached
  *(p_value)  =s_read_byte(ACK);    //read the first byte (MSB)
  *(p_value+1)=s_read_byte(ACK);    //read the second byte (LSB)
  *p_checksum =s_read_byte(noACK);  //read checksum
  return error;
}

/*
//----------------------------------------------------------------------------------
void init_uart()
//----------------------------------------------------------------------------------
//9600 bps @ 11.059 MHz 
{SCON  = 0x52;    
 TMOD  = 0x20;    
 TCON  = 0x69;	  
 TH1   = 0xfd;    
}	*/

//----------------------------------------------------------------------------------------
void calc_sth11(float *p_humidity ,float *p_temperature)
//----------------------------------------------------------------------------------------
// calculates temperature [癈] and humidity [%RH] 
// input :  humi [Ticks] (12 bit) 
//          temp [Ticks] (14 bit)
// output:  humi [%RH]
//          temp [癈]
{ const float C1=-4.0;              // for 12 Bit
  const float C2=+0.0405;           // for 12 Bit
  const float C3=-0.0000028;        // for 12 Bit
  const float T1=+0.01;             // for 14 Bit @ 5V
  const float T2=+0.00008;           // for 14 Bit @ 5V	

  float rh=*p_humidity;             // rh:      Humidity [Ticks] 12 Bit 
  float t=*p_temperature;           // t:       Temperature [Ticks] 14 Bit
  float rh_lin;                     // rh_lin:  Humidity linear
  float rh_true;                    // rh_true: Temperature compensated humidity
  float t_C;                        // t_C   :  Temperature [癈]

  t_C=t*0.01 - 40;                  //calc. temperature from ticks to [癈]
  rh_lin=C3*rh*rh + C2*rh + C1;     //calc. humidity from ticks to [%RH]
  rh_true=(t_C-25)*(T1+T2*rh)+rh_lin;   //calc. temperature compensated humidity [%RH]
  if(rh_true>100)rh_true=100;       //cut if the value is outside of
  if(rh_true<0.1)rh_true=0.1;       //the physical possible range

  *p_temperature=t_C;               //return temperature [癈]
  *p_humidity=rh_true;              //return humidity[%RH]
}

//--------------------------------------------------------------------
float calc_dewpoint(float h,float t)
//--------------------------------------------------------------------
// calculates dew point
// input:   humidity [%RH], temperature [癈]
// output:  dew point [癈]
{ float logEx,dew_point;
  logEx=0.66077+7.5*t/(237.3+t)+(log10(h)-2);
  dew_point = (logEx - 0.66077)*237.3/(0.66077+7.5-logEx);
  return dew_point;
}
														
																		
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
			bSendLED = !bSendLED;
		}
		else
		{
			SetSendFlag(0);	 //发送完后将485置于接收状态
			receCount = 0;   //清接收地址偏移寄存器
			checkoutError = 0;
			bSendLED = 1;
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
		bSendLED = 0;
	}

}   // void CommIntProc()


void showled()
{
	InitDis (); 
	WriteWord (Digit0,DisBuffer[0]); 
	WriteWord (Digit1,DisBuffer[1] | 0x80); 
	WriteWord (Digit2,DisBuffer[2]); 
	WriteWord (Digit3,DisBuffer[3]); 
	WriteWord (Digit4,DisBuffer[4] | 0x80); 
	WriteWord (Digit5,DisBuffer[5]);
}
//定时器0 1ms 中断
void timer0IntProc() interrupt 1
{
	 
	TL0 = TIMER_LOW; 
    TH0 = TIMER_HIGHT;
    dwIntTick++;
	bt1ms = 1;
    c10ms++;
//	ledtick ++;
    if(c10ms >= 10) 
    {
        c10ms = 0;      //10ms计时器清零
        bt10ms = 1;
		  showled();
	
   }

   if(c10ms %3 == 0 )
   {
   		WDT_CONTR=0x35; 
		
   }

		

/*	if(ledtick == 5)
	{	
		showled();
		ledtick = 0;

	} */



}   // void Timer0IntProc()

//外部中断0
void intEx0Proc(void) interrupt 0
{

}

//计数器1中断
void counter1IntProc(void) interrupt 3 using 1
{

}


//定时处理
void timeProc(void)
{
	static uint8 c200ms;
	uint16 j;

	/* SHT 10 */

 	unsigned char error,checksum;
	/* */	


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
				SetSendFlag(0);			 //将485置为接收状态
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
			bRunLED =!bRunLED;
            c200ms = 0;             //清200ms计时器
			
			humi_val.i = 0;
   			temp_val.i = 0;

		//	showled();

			//读取温度及湿度传感器
			 error=0;
		    error+=s_measure((unsigned char*) &humi_val.i,&checksum,HUMI);  //measure humidity
			//	showled();
		    error+=s_measure((unsigned char*) &temp_val.i,&checksum,TEMP);  //measure temperature
		//		showled();
  			 if(error!=0)
			{
			 	 s_connectionreset();                 //in case of an error: connection reset
				humi_val.f = 999.9;
   				temp_val.f = 999.9;
			
			}
		    else
		    {
				 humi_val.f=(float)humi_val.i;                   //converts integer to float
		     	 temp_val.f=(float)temp_val.i;                   //converts integer to float
		      	calc_sth11(&humi_val.f,&temp_val.f);            //calculate humidity, temperature
			//		showled();
		      	dew_point=calc_dewpoint(humi_val.f,temp_val.f); //calculate dew point
		    }

		j =  (uint16)( temp_val.f*10);
		DisBuffer[0] = (uint8)( j%1000/100); 
		DisBuffer[1] = (uint8)( j%100/10); 
		DisBuffer[2] =(uint8)( j%10); 
		
		j =  (uint16) (humi_val.f*10);
		DisBuffer[3] =(uint8)( j%1000/100); 
		DisBuffer[4] =(uint8)( j%100/10); 
		DisBuffer[5] =(uint8)( j%10);

		showled();
			
        }
    }
}   // void TimerProc(void)

//初始化串口
void initUart(void)
{
	//T2 用于波特率 9600
	T2CON = 0x30;
	RCAP2H = 0xff;
	RCAP2L = 0xb8;
	TR2 = 1;

	//偶校验 						
	SCON = 0xd0;
    PCON = 0;
    ES = 1;
}//void initUart(void)

//初始化中断
void initInt(void)
{
	TMOD = 0x51;
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

	EA = 1;		
}   // void initInt(void)

//初始化
void initProg(void)
{	
	initInt();

	SetSendFlag(0);

	 s_connectionreset();
}


void main(void)
{
	 uint8 i;
	 uint16 j;

	initProg();

	//7219
	InitDis (); 
//	WriteWord (DisplayTest,TestMode); 
//	delayX10ms(0x64); 
//	WriteWord (DisplayTest,TextEnd); 

	//喂狗
	WDT_CONTR=0x35; 


	c_1 = 0;
	c_2 = 0;
	c_3 = 0;
	
    //7219

	bRunLED  = 0;
	SIMSTAT = 0;
	for(  i = 0;i<16;i++)
	{
		sendBuf[i]= 0;	
	}
	receCount = 0;
	bSendLED = 1;

	humi_val.f = 0;
   	temp_val.f = 0;

	SetSendFlag(0);


	 ledtick = 0;


//	 nClientID =   (c_1==1)?1:0 + 	(c_2 == 1)?2:0	+ (c_3==1)?4:0 + 1;


	nClientID = 40;

	while(1)
	{
	

		timeProc();

		
		if(	 receCount == 2)
		{
			if(	receBuf[0] == 	   nClientID && checkoutError == 0)
			{
				switch(	 receBuf[1])
				{
					case ACTION_GET:
						 {
						 		for(  i = 0;i<16;i++)
								{
									sendBuf[i]= 0;	
								}

								sendBuf[0] = 0;
								sendBuf[1] = nClientID ;
								sendBuf[2] = ACTION_GET;
								sendBuf[3] =  SIMSTAT;  //模拟机状态

								j =  (uint16)( humi_val.f*10);
							//	j = 1234;
							    sendBuf[4] =(uint8)( j/1000) ;
								sendBuf[5] = (uint8)( j%1000/100) ;
								sendBuf[6] =  (uint8)( j%100/10) ;
								sendBuf[7] =  (uint8)( j%10) ;
								j =  (uint16) (temp_val.f*10);
							//	j = 5678;
								sendBuf[8] =  (uint8)( j/1000) ;
								sendBuf[9] =  (uint8)( j%1000/100) ;
								sendBuf[10] = (uint8)( j%100/10) ;
								sendBuf[11] =    (uint8)( j%10) ;

							
								sendCount = 12;
								beginSend();
						 }
						break;
					default:
						break;
				}
			}
			receCount = 0;
			checkoutError = 0;
		}

	//	if(receCount == 15)
	//	{
		//	bRecvLED = 	  receBuf[1];
	//		receCount = 0;
	//	} 
	/*	if(receCount == 16)
		{
			if(bRecvFlag!=		receBuf[1])
				bRecvFlag = receBuf[1];
			receCount = 0;
			bSignLED = !bSignLED;
		}		 		 */
	/*	if(receCount >0)
		{
			bRecvLED = 1	;
		}
		else
			bRecvLED = 0;	*/
	//	checkComm0Modbus();
	}
}

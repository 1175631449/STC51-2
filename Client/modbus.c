#include "main.h"

//�ֵ�ַ 0 - 255 (ֻȡ��8λ)
//λ��ַ 0 - 255 (ֻȡ��8λ)

/* CRC ��λ�ֽ�ֵ�� */ 
const uint8 code auchCRCHi[] = { 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40 
} ; 
/* CRC��λ�ֽ�ֵ��*/ 
const uint8 code auchCRCLo[] = { 
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 
0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 
0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 
0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 
0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 
0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 
0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 
0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 
0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 
0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 
0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 
0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 
0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 
0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 
0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 
0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 
0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 
0x43, 0x83, 0x41, 0x81, 0x80, 0x40 
} ;

uint8   testCoil;		//���ڲ��� λ��ַ1
uint16  testRegister;	//���ڲ��� ��ַַ16

uint8	localAddr = 1;	//��Ƭ�����ư�ĵ�ַ
uint8	sendCount;		//�����ֽڸ���
uint8	receCount;	    //���յ����ֽڸ���
uint8	sendPosi;	    //����λ��
uint8   iRecvFlag;

uint16 crc16(uint8 *puchMsg, uint16 usDataLen) 
{ 
	uint8 uchCRCHi = 0xFF ; /* ��CRC�ֽڳ�ʼ�� */ 
	uint8 uchCRCLo = 0xFF ; /* ��CRC �ֽڳ�ʼ�� */ 
	uint32 uIndex ; /* CRCѭ���е����� */ 
	while (usDataLen--) /* ������Ϣ������ */ 
	{ 
		uIndex = uchCRCHi ^ *puchMsg++ ; /* ����CRC */ 
		uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex] ; 
		uchCRCLo = auchCRCLo[uIndex] ; 
	} 
	return (uchCRCHi << 8 | uchCRCLo) ; 
}//uint16 crc16(uint8 *puchMsg, uint16 usDataLen)


void SetSendFlag(uint8 value)
{
	uint16 i ;
	if(value)
		b485Send = 0;
	else
		b485Send = 1;	
	i = 5000;
	while(i--);
}

//��ʼ����
void beginSend(void)
{	

	//��Ϊ����
	SetSendFlag(1);
	
	sendPosi = 0;
	if(sendCount > 1)
		sendCount--;
	ACC = sendBuf[0];
	TB8 = P;
	SBUF = sendBuf[0];

}//void beginSend(void)

	 /*
//����Ȧ״̬
void readCoil(void)
{
	uint8 addr;
	uint8 tempAddr;
	uint8 byteCount;
	uint8 bitCount;
	uint16 crcData;
	uint8 position;
	uint8 i,k;
	uint8  result;
	uint16 tempData;
	uint8  exit = 0;
	
	//addr = (receBuf[2]<<8) + receBuf[3];	
	//tempAddr = addr & 0xfff;
	addr = receBuf[3];
	tempAddr = addr;

	//bitCount = (receBuf[4]<<8) + receBuf[5];	//��ȡ��λ����
	bitCount = receBuf[5];

	byteCount = bitCount / 8;					//�ֽڸ���
	if(bitCount%8 != 0)
		byteCount++;
									
	for(k=0;k<byteCount;k++)
	{//�ֽ�λ��
		position = k + 3;
		sendBuf[position] = 0;
		for(i=0;i<8;i++)
		{
			getCoilVal(tempAddr,&tempData);
			
			sendBuf[position] |= tempData << i;
			tempAddr++;
			if(tempAddr >= addr+bitCount)
			{	//����
				exit = 1;
				break;
			}	
		}
		if(exit == 1)
			break;
	}
	
	sendBuf[0] = localAddr;
	sendBuf[1] = 0x01;	
	sendBuf[2] = byteCount;
	byteCount += 3;
	crcData = crc16(sendBuf,byteCount);	
	sendBuf[byteCount] = crcData >> 8;
	byteCount++;
	sendBuf[byteCount] = crcData & 0xff;
	sendCount = byteCount + 1;
	
	beginSend();	
}//void readCoil(void) 

//���Ĵ���
void readRegisters(void)
{
	uint8 addr;
	uint8 tempAddr;
	uint16 result;
	uint16 crcData;
	uint8 readCount;
	uint8 byteCount;
	uint8  finsh;	//1���  0����
	uint16 i;
	uint16 tempData = 0;	
	
	//addr = (receBuf[2]<<8) + receBuf[3];
	//tempAddr = addr & 0xfff;	
	addr = receBuf[3];
	tempAddr = addr;

	//readCount = (receBuf[4]<<8) + receBuf[5];	//Ҫ���ĸ���
	readCount = receBuf[5];

	byteCount = readCount * 2;
	
	for(i=0;i<byteCount;i+=2,tempAddr++)
	{
		getRegisterVal(tempAddr,&tempData);				
		sendBuf[i+3] = tempData >> 8;				   		
		sendBuf[i+4] = tempData & 0xff;			
	}
	
	sendBuf[0] = localAddr;
	sendBuf[1] = 3;
	sendBuf[2] = byteCount;
	byteCount += 3;
	crcData = crc16(sendBuf,byteCount);
	sendBuf[byteCount] = crcData >> 8;
	byteCount++;
	sendBuf[byteCount] = crcData & 0xff;
	
	sendCount = byteCount + 1;
	beginSend();
}//void readRegisters(void)


//ǿ�Ƶ�����Ȧ
void forceSingleCoil(void)
{
	uint8 addr;
	uint8 tempAddr;
	uint16 tempData;
	uint8  onOff;
	uint8 i;
	
	//addr = (receBuf[2]<<8) + receBuf[3];	
	//tempAddr = addr & 0xfff;
	addr = receBuf[3];
	tempAddr = addr;

	//onOff = (receBuf[4]<<8) + receBuf[5];	
	onOff = receBuf[4];
	
	//if(onOff == 0xff00)
	if(onOff == 0xff)
	{	//��ΪON
		tempData = 1;
	} 
	//else if(onOff == 0x0000)
	else if(onOff == 0x00)
	{	//��ΪOFF
		tempData = 0;
	}

	setCoilVal(tempAddr,tempData);	
	
	for(i=0;i<receCount;i++)
	{
		sendBuf[i] = receBuf[i];
	}
	sendCount = receCount;
	beginSend();	
}//void forceSingleCoil(void)


//���ö���Ĵ���
void presetMultipleRegisters(void)
{
	uint8 addr;
	uint8 tempAddr;
	uint8 byteCount;
	uint8 setCount;
	uint16 crcData;
	uint16 tempData;
	uint8  finsh;	//Ϊ1ʱ��� Ϊ0ʱ����
	uint8 i;
	
	//addr = (receBuf[2]<<8) + receBuf[3];
	//tempAddr = addr & 0xfff;
	addr = receBuf[3];
	tempAddr = addr & 0xff;

	//setCount = (receBuf[4]<<8) + receBuf[5];
	setCount = receBuf[5];
	byteCount = receBuf[6];	
	
	for(i=0;i<setCount;i++,tempAddr++)
	{
		tempData = (receBuf[i*2+7]<<8) + receBuf[i*2+8];
	
		setRegisterVal(tempAddr,tempData);			
	}
	
	sendBuf[0] = localAddr;
	sendBuf[1] = 16;
	sendBuf[2] = addr >> 8;
	sendBuf[3] = addr & 0xff;
	sendBuf[4] = setCount >> 8;
	sendBuf[5] = setCount & 0xff;
	crcData = crc16(sendBuf,6);
	sendBuf[6] = crcData >> 8;
	sendBuf[7] = crcData & 0xff;
	sendCount = 8;
	beginSend();	
}//void presetMultipleRegisters(void)



//���uart0����
void checkComm0Modbus(void)
{
	uint16 crcData;
	uint16 tempData;

	if(receCount == 16)
	{
		
	}
	
	return;
	if(receCount > 4)
	{
		switch(receBuf[1])
		{
			case 1://��ȡ��Ȧ״̬(��ȡ�� 16λ����)
			case 3://��ȡ���ּĴ���(һ������)
			case 5://ǿ�Ƶ�����Ȧ
			case 6://���õ����Ĵ���
					if(receCount >= 8)
					{//�������һ������
						//Ӧ�ùرս����ж�
						
						if(receBuf[0]==localAddr && checkoutError==0)
						{
							crcData = crc16(receBuf,6);
							if(crcData == receBuf[7]+(receBuf[6]<<8))
							{//У����ȷ
								if(receBuf[1] == 1)
								{//��ȡ��Ȧ״̬(��ȡ�� 16λ����)
									readCoil();								
								}
								else if(receBuf[1] == 3)
								{//��ȡ���ּĴ���(һ������)
									readRegisters();
								}
								else if(receBuf[1] == 5)
								{//ǿ�Ƶ�����Ȧ
									forceSingleCoil();								
								}
								else if(receBuf[1] == 6)
								{
									//presetSingleRegister();								
								}

							}
						}						
						receCount = 0;	
						checkoutError = 0;
					}
					break;
			
			case 15://���ö����Ȧ
					tempData = receBuf[6]; 
					tempData += 9;	//���ݸ���
					if(receCount >= tempData)
					{
						if(receBuf[0]==localAddr && checkoutError==0)
						{
							crcData = crc16(receBuf,tempData-2);
							if(crcData == (receBuf[tempData-2]<<8)+ receBuf[tempData-1])
							{
								//forceMultipleCoils();			
							}
						}	
						receCount = 0;
						checkoutError = 0;
					}
					break;
			
			case 16://���ö���Ĵ���
					tempData = (receBuf[4]<<8) + receBuf[5];
					tempData = tempData * 2;	//���ݸ���
					tempData += 9;
					if(receCount >= tempData)
					{
						if(receBuf[0]==localAddr && checkoutError==0)
						{
							crcData = crc16(receBuf,tempData-2);
							if(crcData == (receBuf[tempData-2]<<8)+ receBuf[tempData-1])
							{
								presetMultipleRegisters();			
							}
						}	
						receCount = 0;
						checkoutError = 0;
					}
					break;
								
			default:
					break;			
		}
	}
}//void checkComm0(void)

//ȡ��Ȧ״̬ ����0��ʾ�ɹ�
uint16 getCoilVal(uint16 addr,uint16 *tempData)
{
	uint16 result = 0;
	uint16 tempAddr;
	
	tempAddr = addr & 0xfff;
	//ֻȡ��8λ��ַ
	switch(tempAddr & 0xff)
	{
		case 0:
				break;
		case 1:	
				*tempData = testCoil;
				break;
		case 2:		
				break;		
		case 3:
				break;		
		case 4:
				break;		
		case 5:
				break;
		case 6:
				break;			
		case 7:
				break;		
		case 8:
				break;		
		case 9:
				break;		
		case 10:
				break;
		case 11:
				break;
		case 12:
				break;
		case 13:
				break;
		case 14:
				break;
		case 15:
				break;
		case 16:
				break;														
		default:
				break;		
	}	
	
	return result;
}//uint16 getCoilVal(uint16 addr,uint16 *data)


//�趨��Ȧ״̬ ����0��ʾ�ɹ�
uint16 setCoilVal(uint16 addr,uint16 tempData)
{
	uint16 result = 0;
	uint16 tempAddr;
	
	tempAddr = addr & 0xfff;
	
		
	switch(tempAddr & 0xff)
	{
		case 0:
				break;
		case 1:	
				testCoil = tempData;
				break;
		case 2:		
				break;		
		case 3:
				break;		
		case 4:
				break;		
		case 5:
				break;
		case 6:
				break;			
		case 7:
				break;		
		case 8:
				break;		
		case 9:
				break;		
		case 10:
				break;
		case 11:
				break;
		case 12:
				break;
		case 13:
				break;
		case 14:
				break;
		case 15:
				break;
		case 16:
				break;														
		default:
				break;		
	}	


	return result;
}//uint16 setCoilVal(uint16 addr,uint16 data)

//ȡ�Ĵ���ֵ ����0��ʾ�ɹ�
uint16 getRegisterVal(uint16 addr,uint16 *tempData)
{
	uint16 result = 0;
	uint16 tempAddr;
	
	tempAddr = addr & 0xfff;
	
	switch(tempAddr & 0xff)
	{
		case 0:
				break;
		case 1:	
				break;
		case 2:		
				break;		
		case 3:
				break;		
		case 4:
				break;		
		case 5:
				break;
		case 6:
				break;			
		case 7:
				break;		
		case 8:
				break;		
		case 9:
				break;		
		case 10:
				break;
		case 11:
				break;
		case 12:
				break;
		case 13:
				break;
		case 14:
				break;
		case 15:
				break;
		case 16:
				*tempData = testRegister;
				break;														
		default:
				break;		
	}
	
	return result;
}//uint16 getRegisterVal(uint16 addr,uint16 &data)

//���üĴ���ֵ ����0��ʾ�ɹ�
uint16 setRegisterVal(uint16 addr,uint16 tempData)
{
	uint16 result = 0;
	uint16 tempAddr;
	
	tempAddr = addr & 0xfff;
	
	switch(tempAddr & 0xff)
	{
		case 0:
				break;
		case 1:	
				break;
		case 2:		
				break;		
		case 3:
				break;		
		case 4:
				break;		
		case 5:
				break;
		case 6:
				break;			
		case 7:
				break;		
		case 8:
				break;		
		case 9:
				break;		
		case 10:
				break;
		case 11:
				break;
		case 12:
				break;
		case 13:
				break;
		case 14:
				break;
		case 15:
				break;
		case 16:
				testRegister = tempData;
				break;														
		default:
				break;		
	}
	
	return result;
}//uint8 setRegisterVal(uint16 addr,uint16 data)
		 */
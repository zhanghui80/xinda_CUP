#include "ADC.h"
#define N 5


/*----------------------------
��ȡADC���
----------------------------*/
float GetADCResult(BYTE ch)
{
    ADC_CONTR = ADC_POWER | ADC_SPEEDHH | ch | ADC_START;
    _nop_();                        //�ȴ�4��NOP
    _nop_();
    _nop_();
    _nop_();
    while (!(ADC_CONTR & ADC_FLAG));//�ȴ�ADCת�����
    ADC_CONTR &= ~ADC_FLAG;         //Close ADC
    return ADC_RES*50;                 //����ADC���
}

float ADC_MAX(BYTE ch)
{
	int i;
	float temp=0.0,max=0.0;
	for(i=0;i<N;i++)
	{
		temp = GetADCResult(ch);
		if(temp>max)
			max=temp;
	}
//	Send_ADC_Data('M',max);
	return max;
}

/*----------------------------
��ʼ��ADC
----------------------------*/
void InitADC()
{
    P1ASF = 0xff;                   //����P1��ΪAD��
    ADC_RES = 0;                    //�������Ĵ���
    ADC_CONTR = ADC_POWER | ADC_SPEEDHH;
	delay(10);
}

void SendData(BYTE dat)
{
    ACC = dat;                  //��ȡУ��λP (PSW.0)
    SBUF = ACC;                 //д���ݵ�UART���ݼĴ���
	while(TI==0);				//�ȴ����ݷ������
	TI=0;						//�������λ
}

void Send_ADC_Data(BYTE ch,float ADC_Data)
{
	int INT_DATA = (int)(ADC_Data);
	int Data[5];
	Data[0] = INT_DATA/100000%10;
	Data[1] = INT_DATA/1000%10;
	Data[2] = INT_DATA/100%10;
	Data[3] = INT_DATA/10%10;
	Data[4] = INT_DATA/1%10;
	SendData(ch);
	SendData('A');
	SendData('D');
	SendData('C');
	SendData(' ');
	SendData(Data[0]+'0');
	SendData(Data[1]+'0');
	SendData(Data[2]+'0');
	SendData(Data[3]+'0');
	SendData(Data[4]+'0');
	SendData('a');
	SendData('\n');
//	SendData(Data[0]+'0');
//	SendData(Data[1]+'0');
//	SendData(Data[2]+'0');
//	SendData(Data[3]+'0');
//	SendData(Data[4]+'0');
//	SendData(',');
}
void delay(int i)
{
	int temp;
	for(temp=0;temp/10<i;temp++);
}
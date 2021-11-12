#include "STC15F2K60S2.H"
#include "ADC.h"

//����PWM����
float RPWM;
float LPWM;
sbit L_I1 = P2^1;
sbit L_O1 = P2^0;
sbit L_I2 = P2^3;
sbit L_O2 = P2^2;

sbit R_I1 = P2^4;
sbit R_O1 = P2^5;
sbit R_I2 = P2^6;
sbit R_O2 = P2^7;
/********************/
sbit STOP = P3^2;
sbit Bee  =  P5^5;
/**********************/

#define L_Front_RUN L_I1=L_I2=1,L_O1=L_O2=0
#define R_Front_RUN R_I1=R_I2=1,R_O1=R_O2=0
#define L_Back_RUN	L_I1=L_I2=0,L_O1=L_O2=1
#define R_Back_RUN 	R_I1=R_I2=0,R_O1=R_O2=1
#define L_STOP 			L_I1=L_O1=L_I2=L_O2=0
#define R_STOP 			R_I1=R_O1=R_I2=R_O2=0



/*������PWM�Ľṹ��*/
struct{
    float err0;     //��ǰ���
    float err1;     //ǰ1�����
    float P;    //ϵ��
    float dert;     //����
}PID;

struct{
    float left;
    float right;
}PWM;

struct{
    float left;
    float right;
		float mid;
}ADC;

//����һЩ�޷�����������
typedef  unsigned char   u8;
typedef  unsigned int    u16;
typedef  unsigned long   u32;
/*******************/
//����ͣ��
u8 car_state = 0;
u8 temp = 0;
/******************/


//��������PWM���
int T=0;
int Times = 0;
int MID=0;
int SPEED=0;
sbit K1=P0^0;
sbit K2=P0^1;
sbit K3=P0^4;


bit busy;

void PID_RUN(float V_left,float V_right);
void INT_Init();		/***********/
void Timer0Init(void);
void PID_INIT(float,float,float);
void motoProcess(void);
void Send_PWM_Data(BYTE,float);
void Button_Init(void);
void UART_Init(void);
void Send_Der_Data(float,float);
void Delay1ms();		/**********/
/******************** ������**************************/
void main(void)
{
	//��ʱ����PID�ĳ�ʼ��
	PID_INIT(0,0,0);//L,R,P,I,D
	INT_Init();			/**************************/
	Timer0Init();	
	InitADC();
	Button_Init();
	UART_Init();
	while(1){
	if(!K1)	{PID_INIT(40.0, 40.0, 0.23);SPEED=80;break;}//40.0, 40.0, 0.23
	if(!K2)	{PID_INIT(45.0, 45.0, 0.20);SPEED=70;break;}//45.0, 45.0, 0.20
	if(!K3)	{PID_INIT(45.0, 45.0, 0.20);SPEED=99;break;}
	}
	while(1)
	{		
		ADC.left = ADC_MAX(0);
		ADC.right= ADC_MAX(2);
		ADC.mid  = ADC_MAX(1);
		PID_RUN(ADC.left,ADC.right);
		if(ADC.mid>350) MID=1;
		else MID=0;
		
		if(car_state > 1)		//���ڶ��ξ�������ߣ�ͣ��
		{
			break;
		}
	}
}

/*���������*/
void motoProcess()
{
	if(PWM.left <10) LPWM = 11.6-0.16*PWM.left;
	else LPWM=PWM.left;
	if(PWM.right<10) RPWM = 11.6-0.16*PWM.right;
	else RPWM=PWM.right;
}

void Button_Init()
{
	/*����PO_0 P0_1 P0_4Ϊ����̬����Ϊ����*/
	P0M0 = 0X00;
	P0M1 = 0X13;	//0001 0011
	/********************/
	P5M0 = 0X20;
	P5M1 = 0X00;
	Bee = 0;
	STOP = 0;
	/*********************/
}

/*************************************/
void INT_Init()
{
	
	IT0 = 0;		//�����ش���
	EX0 = 1;		//�����ⲿ�ж�
	EA = 1;			//�������ж�
}
/**************************************/

void Timer0Init(void)		//0.1����@24.000MHz
{
	AUXR |= 0x80;		//��ʱ��ʱ��1Tģʽ
	TMOD &= 0xF0;		//���ö�ʱ��ģʽ
	TL0 = 0xA0;		//���ö�ʱ��ֵ
	TH0 = 0xF6;		//���ö�ʱ��ֵ
	TF0 = 0;		//���TF0��־
	TR0 = 1;		//��ʱ��0��ʼ��ʱ
	EA  = 1;
	ET0 = 1;
	TR0 = 1;
}

void UART_Init()
{
//	ACC = P_SW1;
//  ACC &= ~(S1_S0 | S1_S1);    //S1_S0=0 S1_S1=0
//  P_SW1 = ACC;                //(P3.0/RxD, P3.1/TxD)
	SCON = 0x50;		//8λ����,�ɱ䲨����
	AUXR |= 0x40;		//��ʱ��1ʱ��ΪFosc,��1T
	AUXR &= 0xFE;		//����1ѡ��ʱ��1Ϊ�����ʷ�����
	TMOD &= 0x0F;		//�趨��ʱ��1Ϊ16λ�Զ���װ��ʽ
	TL1 = 0xCC;		//�趨��ʱ��ֵ
	TH1 = 0xFF;		//�趨��ʱ��ֵ
	ET1 = 0;		//��ֹ��ʱ��1�ж�
	TR1 = 1;		//������ʱ��1
	ES = 1;			//ʹ�ܴ����ж�
	EA = 1;

}

/*PID��PWM�ĳ�ʼ��*/
void PID_INIT(float PWM_LEFT,float PWM_RIGHT,float PID_P)
{
    PID.err0 = 0.0;
    PID.err1 = 0.0;
    PID.dert = 0.0;
    PID.P = PID_P;
		PWM.left = PWM_LEFT;
    PWM.right= PWM_RIGHT;
		LPWM = PWM_LEFT;
		RPWM = PWM_RIGHT;
}


/*PIDִ�к���������С���˷����������Ե�ʣ�����ֻ����P���ƣ��������PD����*/
void PID_RUN(float ADC_left,float ADC_right)
{
    PID.err1 = PID.err0;
    PID.err0 = ADC_left - ADC_right;
    PID.dert = PID.P*(PID.err0-PID.err1);
    PWM.left -= PID.dert/2;
    PWM.right+= PID.dert/2;
		motoProcess();
}

/*****************************************/
void Int0_run() interrupt 0		//�ⲿ�жϷ�����
{
	if(STOP == 1)
	{
		Delay1ms();
		if(STOP == 1 && temp == 0)
		{
			car_state++;
			if(car_state == 2)
			{
				Bee = 1;
			}
		}
	}
	temp = STOP;
}
/************************************************/

/*�жϺ���*/
void timer0_int (void) interrupt 1
{
	if(T==100)	T=0;
	//�ұߵ�PWM
	if(SPEED&&MID){
		if(T<SPEED) {L_Front_RUN;R_Front_RUN;}
		else		 {L_STOP;R_STOP;}
	}
	else{
	if(T<RPWM)
		if(PWM.right<10) R_Back_RUN;
		else R_Front_RUN;
	else
		R_STOP;
	//��ߵ�PWM
	if(T<LPWM)
		if(PWM.left<10) L_Back_RUN;
		else L_Front_RUN;
	else
		L_STOP;}
	T++;
//UART�������ݽ��ڵ��Բ���ʱʹ��
//*********************************

//	if(Times>4000){
////		Send_PWM_Data('L',LPWM);
////		Send_PWM_Data('R',RPWM);
//		Send_ADC_Data('L',ADC.left);
//		Send_ADC_Data('R',ADC.right);
////		Send_ADC_Data('M',ADC.mid);
//		Send_Der_Data(ADC.left,ADC.right);
//		Times = 0;
//	}
//	Times++;

//*********************************
}

void Send_PWM_Data(BYTE ch,float PWM_Data)
{
	int INT_DATA = (int)PWM_Data;
	int Data[3];
	SendData(ch);
	SendData('P');
	SendData('W');
	SendData('M');
	SendData(' ');
	if(ch=='L')
		if(PWM.left<10)
			SendData('-');
		else SendData('+');
	else if(PWM.right<10)
         SendData('-');
			 else SendData('+');
	Data[0] = INT_DATA/100%10;
	Data[1] = INT_DATA/10%10;
	Data[2] = INT_DATA/1%10;
	SendData(Data[0]+'0');
	SendData(Data[1]+'0');
	SendData(Data[2]+'0');
	SendData('a');
	SendData('\n');
}
void Send_Der_Data(float L,float R)
{
	int INT_Dert = (int)(L-R);
	int Data[3];
	SendData('D');
	SendData('e');
	SendData('r');
	SendData('t');
	SendData(' ');
	if(INT_Dert<0)	{INT_Dert =-INT_Dert;SendData('-');}
	else						{INT_Dert = INT_Dert;SendData('+');}
	Data[0] = INT_Dert/100%10;
	Data[1] = INT_Dert/10%10;
	Data[2] = INT_Dert/1%10;
	SendData(Data[0]+'0');
	SendData(Data[1]+'0');
	SendData(Data[2]+'0');
	SendData('a');
	SendData('\n');
}

void Delay1ms()		//@24.000MHz
{
	unsigned char i, j;

	i = 24;
	j = 85;
	do
	{
		while (--j);
	} while (--i);
}
#include "STC15F2K60S2.H"
#include "ADC.h"

//左右PWM控制
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



/*这里是PWM的结构体*/
struct{
    float err0;     //当前误差
    float err1;     //前1次误差
    float P;    //系数
    float dert;     //增量
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

//定义一些无符号整型数据
typedef  unsigned char   u8;
typedef  unsigned int    u16;
typedef  unsigned long   u32;
/*******************/
//用于停车
u8 car_state = 0;
u8 temp = 0;
/******************/


//变量用于PWM输出
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
/******************** 主函数**************************/
void main(void)
{
	//定时器和PID的初始化
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
		
		if(car_state > 1)		//当第二次经过起点线，停车
		{
			break;
		}
	}
}

/*电机处理函数*/
void motoProcess()
{
	if(PWM.left <10) LPWM = 11.6-0.16*PWM.left;
	else LPWM=PWM.left;
	if(PWM.right<10) RPWM = 11.6-0.16*PWM.right;
	else RPWM=PWM.right;
}

void Button_Init()
{
	/*配置PO_0 P0_1 P0_4为高阻态，作为输入*/
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
	
	IT0 = 0;		//上升沿触发
	EX0 = 1;		//开启外部中断
	EA = 1;			//开启总中断
}
/**************************************/

void Timer0Init(void)		//0.1毫秒@24.000MHz
{
	AUXR |= 0x80;		//定时器时钟1T模式
	TMOD &= 0xF0;		//设置定时器模式
	TL0 = 0xA0;		//设置定时初值
	TH0 = 0xF6;		//设置定时初值
	TF0 = 0;		//清除TF0标志
	TR0 = 1;		//定时器0开始计时
	EA  = 1;
	ET0 = 1;
	TR0 = 1;
}

void UART_Init()
{
//	ACC = P_SW1;
//  ACC &= ~(S1_S0 | S1_S1);    //S1_S0=0 S1_S1=0
//  P_SW1 = ACC;                //(P3.0/RxD, P3.1/TxD)
	SCON = 0x50;		//8位数据,可变波特率
	AUXR |= 0x40;		//定时器1时钟为Fosc,即1T
	AUXR &= 0xFE;		//串口1选择定时器1为波特率发生器
	TMOD &= 0x0F;		//设定定时器1为16位自动重装方式
	TL1 = 0xCC;		//设定定时初值
	TH1 = 0xFF;		//设定定时初值
	ET1 = 0;		//禁止定时器1中断
	TR1 = 1;		//启动定时器1
	ES = 1;			//使能串口中断
	EA = 1;

}

/*PID和PWM的初始化*/
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


/*PID执行函数，由于小车运发出现问题的缘故，这里只用了P控制，建议采用PD控制*/
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
void Int0_run() interrupt 0		//外部中断服务函数
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

/*中断函数*/
void timer0_int (void) interrupt 1
{
	if(T==100)	T=0;
	//右边的PWM
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
	//左边的PWM
	if(T<LPWM)
		if(PWM.left<10) L_Back_RUN;
		else L_Front_RUN;
	else
		L_STOP;}
	T++;
//UART传输数据仅在调试参数时使用
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
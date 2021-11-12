#ifndef __ADC_H
#define __ADC_H

#include "STC15F2K60S2.H"
#include "intrins.h"

#define FOSC 11059200L          //系统频率
#define BAUD 115200             //串口波特率

#define NONE_PARITY     0       //无校验
#define ODD_PARITY      1       //奇校验
#define EVEN_PARITY     2       //偶校验
#define MARK_PARITY     3       //标记校验
#define SPACE_PARITY    4       //空白校验


#define ADC_POWER   0x80            //ADC电源控制位
#define ADC_FLAG    0x10            //ADC完成标志
#define ADC_START   0x08            //ADC起始控制位
#define ADC_SPEEDLL 0x00            //540个时钟
#define ADC_SPEEDL  0x20            //360个时钟
#define ADC_SPEEDH  0x40            //180个时钟
#define ADC_SPEEDHH 0x60            //90个时钟

#define PARITYBIT EVEN_PARITY   //定义校验位

#define S1_S0 0x40              //P_SW1.6
#define S1_S1 0x80              //P_SW1.7




typedef unsigned char BYTE;
typedef unsigned int WORD;

void InitADC();
float GetADCResult(BYTE ch);
float ADC_MAX(BYTE ch);
void SendData(BYTE dat);
void Send_ADC_Data(BYTE ch,float ADC_Data);
void delay(int);
#endif  


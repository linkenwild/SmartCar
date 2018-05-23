/*********************2017-2017, NJUT, Edu.********************* 
FileName: usart.c 
Author:  �ﶬ÷       Version :  1.0        Date: 2017.06.10
Description:    usart0����  ����5��ȡ����,���յ�\r ���ӡ�����    
Version:         1.0 
History:         
      <author>  <time>   <version >   <desc> 
      Sundm    17/06/10     1.0     �ļ�����   
  *          Smartcar Board Key Pin assignment
  *          =============================
  *          +-----------------------------------------------------------+
  *          |                     Pin assignment                        |
  *          +-----------------------------+-----------------------------+
  *          |      FunctionPin            |     Port & Pin              |
  *          +-----------------------------+-----------------------------+
  *          |      USART0_TX              |        D7                   |
  *          |      USART0_RX              |        D6                   |
  *          +-----------------------------+-----------------------------+
***************************************************************/ 
#include "fsl_uart.h"
#include <stdio.h>
#include <stdarg.h>
#include "fsl_port.h"

#include "usart.h"
#include "led.h"

/* UART instance and clock */
#define BOARD_UART UART0
//#define BOARD_UART_CLKSRC kCLOCK_BusClk
#define BOARD_UART_CLKSRC kCLOCK_CoreSysClk
#define BOARD_UART_IRQn UART0_RX_TX_IRQn
#define BOARD_UART_IRQHandler UART0_RX_TX_IRQHandler

/*******************************************************************************
 * Variables
 ******************************************************************************/

uint8_t g_tipString[] =
    "Uart functional API interrupt example\r\nBoard receives characters then sends them out\r\nNow please input and press Enter key to end:\r\n";

	#define RX_BUFFER_SIZE 256

uint8_t RxBuffer[RX_BUFFER_SIZE];
volatile uint16_t txIndex; /* Index of the data to send out. */
volatile uint16_t rxIndex; /* Index of the memory to save new arrived data. */

static volatile bool g_UsartRevDataFlag = false;

/*******************************************************************************
* Function Name  : BOARD_UART_IRQHandler
* Description    : �����жϣ��������ݣ����ڻ�ȡ����,���յ�\r\n�� ��־λ��λ
* Input          : None
* Output         : None
* Return         : None
 ******************************************************************************/

void BOARD_UART_IRQHandler(void)
{
    uint8_t data;

    /* If new data arrived. */
    if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(BOARD_UART))
    {
        data = UART_ReadByte(BOARD_UART);

        /* If ring buffer is not full, add data to ring buffer. */
        if ( 0x0D == data )
        {
            RxBuffer[rxIndex] = data;
            rxIndex++;
            g_UsartRevDataFlag = true;
        }
        else
        {
                RxBuffer[rxIndex] = data;
                rxIndex++;
        }
    }
}

/*******************************************************************************
* Function Name  : Uart5PinConfig �ڲ�����
* Description    : USART5 Pin��ʼ��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void UartPinConfig(void)
{
    /* Initialize UART5 pins below */
    /* Ungate the port clock */
    CLOCK_EnableClock(kCLOCK_PortD);

    /* Affects PORTE_PCR8 register */
    PORT_SetPinMux(PORTD, 6U, kPORT_MuxAlt3);
    /* Affects PORTE_PCR9 register */
    PORT_SetPinMux(PORTD, 7U, kPORT_MuxAlt3);
}

/*******************************************************************************
* Function Name  : Uart5Config
* Description    : USART5��ʼ��,115200,��У�飬8���ݣ�1ֹͣ���жϽ���
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UartConfig(void)
{
    uart_config_t config;
    
    UartPinConfig();
    
    /*
     * config.baudRate_Bps = 115200U;
     * config.parityMode = kUART_ParityDisabled;
     * config.stopBitCount = kUART_OneStopBit;
     * config.txFifoWatermark = 0;
     * config.rxFifoWatermark = 1;
     * config.enableTx = false;
     * config.enableRx = false;
     */
    UART_GetDefaultConfig(&config);
    config.baudRate_Bps = 115200;
    config.enableTx = true;
    config.enableRx = true;

    UART_Init(BOARD_UART, &config, CLOCK_GetFreq(BOARD_UART_CLKSRC));

    /* Send g_tipString out. */
    UART_WriteBlocking(BOARD_UART, g_tipString, sizeof(g_tipString) / sizeof(g_tipString[0]));

    /* Enable RX interrupt. */
    UART_EnableInterrupts(BOARD_UART, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable);	
    EnableIRQ(BOARD_UART_IRQn);
}

/*******************************************************************************
* Function Name  : PutChar
* Description    : USART1����һ�ֽ�
* Input          : ch Ҫ���͵��ֽ�
* Output         : None
* Return         : None
*******************************************************************************/
void PutChar(uint8_t ch)
{
	UART_WriteByte(BOARD_UART, ch);
}

/*******************************************************************************
* Function Name  : PutString
* Description    : USART1�����ַ���
* Input          : str Ҫ���͵��ַ���
* Output         : None
* Return         : None
*******************************************************************************/
void PutString(uint8_t *str)
{
	UART_WriteBlocking(BOARD_UART, str, strlen((char const*)str));
}

/*******************************************************************************
* Function Name  : myitoa �ڲ�����
* Description    : ת���ַ�������ʽ���ַ�����
* Input          : buf Ҫת�����ַ��� dataת������
* Output         : None
* Return         : None
*******************************************************************************/
void myitoa(int data,char *buf )
{
	int temp,j=0,i=0;
 	while(data)    //�����������֣����Լ�ȡ�����ֲ��ԣ���123�������ַ������е�ֵΪ321
 	{
    	buf[i++] = data%10+'0';//��ת����������ַ�������ַ�������
    	data = data/10;    //ɾ���Ѿ�ת�������֣�Ϊȡ��һ����������׼��
 	}
 	buf[i--]='\0';    //ת�������Ҫ���ַ���������һ���ַ���������־'/0'��������һ���ַ���
 	while( j < i )    //�ո�ת���õ��ַ���������ı��������ת����
 	{
  		temp = buf[j];
  		buf[j] = buf[i];
  		buf[i] = temp;
  		i--,j++;
 	}
}

/*******************************************************************************
* Function Name  : UartPrintf 
* Description    : ���ڸ�ʽ����ӡ
* Input          : format ��ʽ���ַ� ��������printfʹ�ø�ʽ��ͬ��%c %s %d 
* Output         : None
* Return         : None
*******************************************************************************/
void UartPrintf(const char *format, ...)
{
	va_list ap;
	char c,nc;


	va_start(ap, format);	 //���ҵ��󽫲�����ջ,apָ��format
	while (c = *format++)		
	{
		
		if(c == '%'&&(nc = *format++) != '\0')
		{
			switch(nc)
	  		{
          		case 'c':  //���1���ַ�
		 	{
               		        char ch = va_arg(ap, int);  //���ú�ջλ��+1
               		        PutChar(ch);        //�����ַ�
               		        break;
          		}
          		case 's': //����ַ���
			{
               		        char *p = va_arg(ap, char *);
               		        PutString((uint8_t *)p);    //�����ַ���
               		        break;
          		}
			case 'd':
			{
				int data = va_arg(ap,int);
       				char buf[16];
       				myitoa(data,buf);
       				PutString((uint8_t *)buf);//�����ַ���
       				break;
			}
          		default:
               		    PutChar(nc); //�����ַ�
        	}
	    }else
	    {PutChar(c);}
	}
     va_end(ap);	//�ر�ָ��
}

/*******************************************************************************
* Function Name  : GetChar 
* Description    : ���ڽ���һ���ַ�
* Input          : None
* Output         : None
* Return         : ���յ����ַ�
*******************************************************************************/
uint8_t GetChar(void)
{ 
    uint8_t ch;	
    while(!(kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(BOARD_UART))    
    ch = UART_ReadByte(BOARD_UART);   									 
    return ch;   
}

/*******************************************************************************
* Function Name  : fputc �ڲ�����
* Description    : �򴮿�д�뵥���ַ���printf��������ô˺���,���ø�ʽʡ��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int fputc(int ch)
{
	PutChar( ch);  
	return ch;
}

/*******************************************************************************
* Function Name  : fgetc �ڲ�����
* Description    : ���ڻ�ȡ�����ַ���scanf��������ô˺��������ø�ʽʡ��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int fgetc(void)
{
    int ch;
    ch = GetChar();   									 
    return ch;   
}	

/*******************************************************************************
* Function Name  : printuart
* Description    : ���ڻ�ȡ����,���յ�\r\n���ӡ�����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void printuart(void)
{
  if( true == g_UsartRevDataFlag)
  {
      PutString(RxBuffer);
      g_UsartRevDataFlag = false;
      memset(RxBuffer, 0x00, RX_BUFFER_SIZE);
      rxIndex = 0;   
      PutString("\r\n");
  }
}	

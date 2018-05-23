/*********************2017-2017, NJUT, Edu.********************* 
FileName: usart.c 
Author:  孙冬梅       Version :  1.0        Date: 2017.06.10
Description:    usart0驱动  串口5获取数据,当收到\r 后打印后清空    
Version:         1.0 
History:         
      <author>  <time>   <version >   <desc> 
      Sundm    17/06/10     1.0     文件创建   
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
* Description    : 串口中断，接收数据，串口获取数据,当收到\r\n后 标志位置位
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
* Function Name  : Uart5PinConfig 内部函数
* Description    : USART5 Pin初始化
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
* Description    : USART5初始化,115200,无校验，8数据，1停止，中断接收
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
* Description    : USART1发送一字节
* Input          : ch 要发送的字节
* Output         : None
* Return         : None
*******************************************************************************/
void PutChar(uint8_t ch)
{
	UART_WriteByte(BOARD_UART, ch);
}

/*******************************************************************************
* Function Name  : PutString
* Description    : USART1发送字符串
* Input          : str 要发送的字符串
* Output         : None
* Return         : None
*******************************************************************************/
void PutString(uint8_t *str)
{
	UART_WriteBlocking(BOARD_UART, str, strlen((char const*)str));
}

/*******************************************************************************
* Function Name  : myitoa 内部函数
* Description    : 转换字符串，格式化字符串用
* Input          : buf 要转换的字符串 data转换长度
* Output         : None
* Return         : None
*******************************************************************************/
void myitoa(int data,char *buf )
{
	int temp,j=0,i=0;
 	while(data)    //反序生成数字，可自己取个数字测试，如123，反序字符数组中的值为321
 	{
    	buf[i++] = data%10+'0';//将转换后的数字字符存放在字符数组中
    	data = data/10;    //删除已经转换的数字，为取下一个数字做好准备
 	}
 	buf[i--]='\0';    //转换完后还需要在字符数组后面加一个字符串结束标志'/0'，代表是一个字符串
 	while( j < i )    //刚刚转换好的字符串是逆序的必须把它反转过来
 	{
  		temp = buf[j];
  		buf[j] = buf[i];
  		buf[i] = temp;
  		i--,j++;
 	}
}

/*******************************************************************************
* Function Name  : UartPrintf 
* Description    : 串口格式化打印
* Input          : format 格式化字符 。。。与printf使用格式相同，%c %s %d 
* Output         : None
* Return         : None
*******************************************************************************/
void UartPrintf(const char *format, ...)
{
	va_list ap;
	char c,nc;


	va_start(ap, format);	 //从右到左将参数入栈,ap指向format
	while (c = *format++)		
	{
		
		if(c == '%'&&(nc = *format++) != '\0')
		{
			switch(nc)
	  		{
          		case 'c':  //输出1个字符
		 	{
               		        char ch = va_arg(ap, int);  //调用后栈位置+1
               		        PutChar(ch);        //发送字符
               		        break;
          		}
          		case 's': //输出字符串
			{
               		        char *p = va_arg(ap, char *);
               		        PutString((uint8_t *)p);    //发送字符串
               		        break;
          		}
			case 'd':
			{
				int data = va_arg(ap,int);
       				char buf[16];
       				myitoa(data,buf);
       				PutString((uint8_t *)buf);//发送字符串
       				break;
			}
          		default:
               		    PutChar(nc); //发送字符
        	}
	    }else
	    {PutChar(c);}
	}
     va_end(ap);	//关闭指针
}

/*******************************************************************************
* Function Name  : GetChar 
* Description    : 串口接收一个字符
* Input          : None
* Output         : None
* Return         : 接收到的字符
*******************************************************************************/
uint8_t GetChar(void)
{ 
    uint8_t ch;	
    while(!(kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(BOARD_UART))    
    ch = UART_ReadByte(BOARD_UART);   									 
    return ch;   
}

/*******************************************************************************
* Function Name  : fputc 内部函数
* Description    : 向串口写入单个字符。printf函数需调用此函数,调用格式省略
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
* Function Name  : fgetc 内部函数
* Description    : 串口获取单个字符。scanf函数需调用此函数，调用格式省略
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
* Description    : 串口获取数据,当收到\r\n后打印后清空
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

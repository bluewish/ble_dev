#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_uart.h"
#include "hal_lcd.h"
#include "SerialApp.h"
#include "stdarg.h"
#include "att.h"
#include "gatt.h"
#include "AmoMcu_SimpleUart.h"

//static uint8 sendMsgTo_TaskID;

/*
串口设备初始化，
必须在使用串口打印之前调用该函数进行uart初始化
*/
void SerialApp_Init( uint8 taskID )
{
  //调用uart初始化代码
  serialAppInitTransport();
  //记录任务函数的taskID，备用
  //sendMsgTo_TaskID = taskID;
}

/*
uart初始化代码，配置串口的波特率、流控制等
*/
void serialAppInitTransport( )
{
  halUARTCfg_t uartConfig;

  // configure UART
  uartConfig.configured           = TRUE;
  uartConfig.baudRate             = SBP_UART_BR;//波特率
  uartConfig.flowControl          = SBP_UART_FC;//流控制
  uartConfig.flowControlThreshold = SBP_UART_FC_THRESHOLD;//流控制阈值，当开启flowControl时，该设置有效
  uartConfig.rx.maxBufSize        = SBP_UART_RX_BUF_SIZE;//uart接收缓冲区大小
  uartConfig.tx.maxBufSize        = SBP_UART_TX_BUF_SIZE;//uart发送缓冲区大小
  uartConfig.idleTimeout          = SBP_UART_IDLE_TIMEOUT;
  uartConfig.intEnable            = SBP_UART_INT_ENABLE;//是否开启中断
  uartConfig.callBackFunc         = sbpSerialAppCallback;//uart接收回调函数，在该函数中读取可用uart数据

  // start UART
  // Note: Assumes no issue opening UART port.
  (void)HalUARTOpen( SBP_UART_PORT, &uartConfig );

  return;
}
  uint16 numBytes;
/*
bStatus_t sbpSerialAppSendNoti(uint8 *pBuffer,uint16 length)
{

  extern gattAttribute_t simpleProfileAttrTbl[];
  uint8 len;
  if(length > 20)
    len = 20;
  else
    len = length;
  static attHandleValueNoti_t pReport;
  //pReport.handle=0x2E;
  pReport.handle=simpleProfileAttrTbl[0].handle;;
  pReport.len = len;
  osal_memcpy(pReport.value, pBuffer, len);
  return GATT_Notification( 0, &pReport, FALSE );

  return 0;
}
  */
  
/*
uart接收回调函数
当我们通过pc向开发板发送数据时，会调用该函数来接收
*/
void sbpSerialAppCallback(uint8 port, uint8 event)
{
  uint8  pktBuffer[SBP_UART_RX_BUF_SIZE] = {0};
  // unused input parameter; PC-Lint error 715.
  bStatus_t ret;
  
  (void)event;
  HalLcdWriteString("Data form my UART:", HAL_LCD_LINE_4 );
  //返回可读的字节
  if ( (numBytes = Hal_UART_RxBufLen(port)) > 0 ){
  	//读取全部有效的数据，这里可以一个一个读取，以解析特定的命令
	(void)HalUARTRead (port, pktBuffer, numBytes);
	HalLcdWriteString((char*)pktBuffer, HAL_LCD_LINE_5 );
    {
        static uint8 skKey = 0;
        sprintf((char *)pktBuffer, "I am ffff %d", skKey++);
        numBytes = osal_strlen((char *)pktBuffer);
        SK_SetParameter( SK_UART_ATTR, numBytes, pktBuffer );


	}

  }
  
}
/*
打印一段数据
pBuffer可以包含0x00
*/
void sbpSerialAppWrite(uint8 *pBuffer, uint16 length)
{
	HalUARTWrite (SBP_UART_PORT, pBuffer, length);
}
/*
打印一个字符串
str不可以包含0x00，除非结尾
*/
void SerialPrintString(uint8 str[])
{
  HalUARTWrite (SBP_UART_PORT, str, osal_strlen((char*)str));
}
/*
打印指定的格式的数值
参数
title,前缀字符串
value,需要显示的数值
format,需要显示的进制，十进制为10,十六进制为16
*/
void SerialPrintValue(char *title, uint16 value, uint8 format)
{
  uint8 tmpLen;
  uint8 buf[256];
  uint32 err;

  tmpLen = (uint8)osal_strlen( (char*)title );
  osal_memcpy( buf, title, tmpLen );
  buf[tmpLen] = ' ';
  err = (uint32)(value);
  _ltoa( err, &buf[tmpLen+1], format );
  SerialPrintString(buf);		
}

//add by AmoMcu       2014-05-01   格式化串口打印
void SerialPrintFormat(const char* pcFormat, ...)
{
#if (HAL_UART == TRUE)  
  char sprint_buf[256] = {0};		    
  va_list args;
  int n;

  //打印时间
  uint32 time = osal_GetSystemClock();
  sprintf(sprint_buf, "[%6ld.%03ld]: ", time/1000U, time%1000U);

  va_start(args, pcFormat);
  n = vsprintf(sprint_buf+strlen(sprint_buf), pcFormat, args);
  va_end(args);

  HalUARTWrite (SBP_UART_PORT, sprint_buf, osal_strlen((char*)sprint_buf));
#endif  
}

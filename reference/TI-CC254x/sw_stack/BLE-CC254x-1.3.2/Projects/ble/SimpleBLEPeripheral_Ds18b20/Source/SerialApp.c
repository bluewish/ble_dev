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
�����豸��ʼ����
������ʹ�ô��ڴ�ӡ֮ǰ���øú�������uart��ʼ��
*/
void SerialApp_Init( uint8 taskID )
{
  //����uart��ʼ������
  serialAppInitTransport();
  //��¼��������taskID������
  //sendMsgTo_TaskID = taskID;
}

/*
uart��ʼ�����룬���ô��ڵĲ����ʡ������Ƶ�
*/
void serialAppInitTransport( )
{
  halUARTCfg_t uartConfig;

  // configure UART
  uartConfig.configured           = TRUE;
  uartConfig.baudRate             = SBP_UART_BR;//������
  uartConfig.flowControl          = SBP_UART_FC;//������
  uartConfig.flowControlThreshold = SBP_UART_FC_THRESHOLD;//��������ֵ��������flowControlʱ����������Ч
  uartConfig.rx.maxBufSize        = SBP_UART_RX_BUF_SIZE;//uart���ջ�������С
  uartConfig.tx.maxBufSize        = SBP_UART_TX_BUF_SIZE;//uart���ͻ�������С
  uartConfig.idleTimeout          = SBP_UART_IDLE_TIMEOUT;
  uartConfig.intEnable            = SBP_UART_INT_ENABLE;//�Ƿ����ж�
  uartConfig.callBackFunc         = sbpSerialAppCallback;//uart���ջص��������ڸú����ж�ȡ����uart����

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
uart���ջص�����
������ͨ��pc�򿪷��巢������ʱ������øú���������
*/
void sbpSerialAppCallback(uint8 port, uint8 event)
{
  uint8  pktBuffer[SBP_UART_RX_BUF_SIZE] = {0};
  // unused input parameter; PC-Lint error 715.
  bStatus_t ret;
  
  (void)event;
  HalLcdWriteString("Data form my UART:", HAL_LCD_LINE_4 );
  //���ؿɶ����ֽ�
  if ( (numBytes = Hal_UART_RxBufLen(port)) > 0 ){
  	//��ȡȫ����Ч�����ݣ��������һ��һ����ȡ���Խ����ض�������
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
��ӡһ������
pBuffer���԰���0x00
*/
void sbpSerialAppWrite(uint8 *pBuffer, uint16 length)
{
	HalUARTWrite (SBP_UART_PORT, pBuffer, length);
}
/*
��ӡһ���ַ���
str�����԰���0x00�����ǽ�β
*/
void SerialPrintString(uint8 str[])
{
  HalUARTWrite (SBP_UART_PORT, str, osal_strlen((char*)str));
}
/*
��ӡָ���ĸ�ʽ����ֵ
����
title,ǰ׺�ַ���
value,��Ҫ��ʾ����ֵ
format,��Ҫ��ʾ�Ľ��ƣ�ʮ����Ϊ10,ʮ������Ϊ16
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

//add by AmoMcu       2014-05-01   ��ʽ�����ڴ�ӡ
void SerialPrintFormat(const char* pcFormat, ...)
{
#if (HAL_UART == TRUE)  
  char sprint_buf[256] = {0};		    
  va_list args;
  int n;

  //��ӡʱ��
  uint32 time = osal_GetSystemClock();
  sprintf(sprint_buf, "[%6ld.%03ld]: ", time/1000U, time%1000U);

  va_start(args, pcFormat);
  n = vsprintf(sprint_buf+strlen(sprint_buf), pcFormat, args);
  va_end(args);

  HalUARTWrite (SBP_UART_PORT, sprint_buf, osal_strlen((char*)sprint_buf));
#endif  
}

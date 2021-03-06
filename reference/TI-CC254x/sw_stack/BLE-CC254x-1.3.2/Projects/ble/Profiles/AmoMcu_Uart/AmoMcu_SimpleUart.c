/**************************************************************************************************
  Filename:       AmoMcu_SimpleUart.c
  Revised:        $Date: 2013-05-06 13:33:47 -0700 (Mon, 06 May 2013) $
  Revision:       $Revision: 34153 $

  Description:    Simple Keys Profile


  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "AmoMcu_SimpleUart.h"
#include "Peripheral.h"
#include "stdio.h"
#include "HAL_led.h"
#include "HAL_uart.h"

#define HAL_UART_TRANS  1 


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        5

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// SK Service UUID: 0x1800
CONST uint8 skServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SK_SERV_UUID), HI_UINT16(SK_SERV_UUID)
};

// Key Pressed UUID: 0x1801
CONST uint8 keyPressedUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SK_KEYPRESSED_UUID), HI_UINT16(SK_KEYPRESSED_UUID)
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * Profile Attributes - variables
 */

// SK Service attribute
static CONST gattAttrType_t skService = { ATT_BT_UUID_SIZE, skServUUID };

// Keys Pressed Characteristic Properties
static uint8 skCharProps = GATT_PROP_WRITE | GATT_PROP_NOTIFY;

// Key Pressed State Characteristic
#ifdef HAL_UART_TRANS
#define MAX_NUM_SEND_BYTES  128
static uint8 skKeyPressed[MAX_NUM_SEND_BYTES] = {0};
static uint8 skKeyPressed_len = 0;
#else
static uint8 skKeyPressed = 0;
#endif

// Key Pressed Characteristic Configs
static gattCharCfg_t skConfig[GATT_MAX_NUM_CONN];

// Key Pressed Characteristic User Description
static uint8 skCharUserDesp[16] = "Key Press State\0";


/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t simplekeysAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = 
{
  // Simple Keys Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&skService                       /* pValue */
  },

    // Characteristic Declaration for Keys
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &skCharProps 
    },

      // Characteristic Value- Key Pressed
      { 
        { ATT_BT_UUID_SIZE, keyPressedUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
#ifdef HAL_UART_TRANS
        skKeyPressed 
#else
        &skKeyPressed 
#endif
      },

      // Characteristic configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)skConfig 
      },

      // Characteristic User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        skCharUserDesp 
      },      
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 sk_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t sk_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );

static void sk_HandleConnStatusCB( uint16 connHandle, uint8 changeType );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// SK Service Callbacks
CONST gattServiceCBs_t skCBs =
{
  sk_ReadAttrCB,  // Read callback function pointer
  sk_WriteAttrCB, // Write callback function pointer
  NULL            // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SK_AddService
 *
 * @brief   Initializes the Simple Key service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t SK_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, skConfig );

  // Register with Link DB to receive link status change callback
  VOID linkDB_Register( sk_HandleConnStatusCB );  
  
  
  if ( services & SK_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( simplekeysAttrTbl, 
                                          GATT_NUM_ATTRS( simplekeysAttrTbl ),
                                          &skCBs );
  }

  return ( status );
}
/*
bStatus_t sbpSerialAppSendNoti(uint8 *pBuffer,uint16 length)
{
  uint8 len;
  uint16 handle; 
  if(length > 20)
    len = 20;
  else
    len = length;
  static attHandleValueNoti_t pReport;
  //pReport.handle=0x2E;

  GAPRole_GetParameter( GAPROLE_CONNHANDLE, &handle );   
  
  pReport.handle=handle;;
  pReport.len = len;
  osal_memcpy(pReport.value, pBuffer, len);
  return GATT_Notification( 0, &pReport, FALSE );
}

*/

/*********************************************************************
 * @fn      SK_SetParameter
 *
 * @brief   Set a Simple Key Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   pValue - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t SK_SetParameter( uint8 param, uint8 len, void *pValue )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case SK_KEY_ATTR:
      if ( len == sizeof ( uint8 ) ) 
      {
#ifdef HAL_UART_TRANS
        static uint8 skKey = 0;
/*
        skKeyPressed_len = MAX_NUM_SEND_BYTES;
        for(int i = 0; i<skKeyPressed_len; i++ )
        {
            skKeyPressed[i] = skKey++;
        }        
*/
        sprintf((char *)skKeyPressed, "I am AmoMcu %d", skKey++);
        skKeyPressed_len = osal_strlen((char *)skKeyPressed);
        // See if Notification/Indication has been enabled
  #if 0        
        GATTServApp_ProcessCharCfg( skConfig, skKeyPressed, FALSE, 
                                    simplekeysAttrTbl, GATT_NUM_ATTRS( simplekeysAttrTbl ),
                                    INVALID_TASK_ID );
  #else        
        //sbpSerialAppSendNoti(skKeyPressed, 2);
        GATTServApp_ProcessCharCfg( skConfig, skKeyPressed, FALSE, 
                                    simplekeysAttrTbl, GATT_NUM_ATTRS( simplekeysAttrTbl ),
                                    INVALID_TASK_ID );
  #endif
#else
        skKeyPressed = *((uint8*)pValue);
        // See if Notification/Indication has been enabled
        GATTServApp_ProcessCharCfg( skConfig, &skKeyPressed, FALSE, 
                                    simplekeysAttrTbl, GATT_NUM_ATTRS( simplekeysAttrTbl ),
                                    INVALID_TASK_ID );
#endif
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case SK_UART_ATTR:
      if ( len > 0) 
      {
        if(len > MAX_NUM_SEND_BYTES)
            len = MAX_NUM_SEND_BYTES;
            
#ifdef HAL_UART_TRANS
        static uint8 skKey = 0;
/*
        skKeyPressed_len = MAX_NUM_SEND_BYTES;
        for(int i = 0; i<skKeyPressed_len; i++ )
        {
            skKeyPressed[i] = skKey++;
        }        
*/
        //osal_memcmp(skKeyPressed, pValue, len);   //  此函数不生效， 不知为何
        sprintf((char *)skKeyPressed, "%s", pValue);
        skKeyPressed_len = len;

        // See if Notification/Indication has been enabled
  #if 0
        GATTServApp_ProcessCharCfg( skConfig, skKeyPressed, FALSE, 
                                    simplekeysAttrTbl, GATT_NUM_ATTRS( simplekeysAttrTbl ),
                                    INVALID_TASK_ID );
  #else        
        //sbpSerialAppSendNoti(skKeyPressed, 2);
        GATTServApp_ProcessCharCfg( skConfig, skKeyPressed, FALSE, 
                                    simplekeysAttrTbl, GATT_NUM_ATTRS( simplekeysAttrTbl ),
                                    INVALID_TASK_ID );
  #endif
#else
        skKeyPressed = *((uint8*)pValue);
        // See if Notification/Indication has been enabled
        GATTServApp_ProcessCharCfg( skConfig, &skKeyPressed, FALSE, 
                                    simplekeysAttrTbl, GATT_NUM_ATTRS( simplekeysAttrTbl ),
                                    INVALID_TASK_ID );
#endif
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;


      
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn      SK_GetParameter
 *
 * @brief   Get a Simple Key Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   pValue - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t SK_GetParameter( uint8 param, void *pValue )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case SK_KEY_ATTR:
#ifdef HAL_UART_TRANS
#else
      *((uint8*)pValue) = skKeyPressed;
#endif
      break;
      
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          sk_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 *
 * @return      Success or Failure
 */
static uint8 sk_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
  bStatus_t status = SUCCESS;
 
  // Make sure it's not a blob operation (no attributes in the profile are long
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }
 
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles this type for reads

      // simple keys characteristic does not have read permissions, but because it
      //   can be sent as a notification, it must be included here
      case SK_KEYPRESSED_UUID:
#ifdef HAL_UART_TRANS
        *pLen = skKeyPressed_len;
        osal_memcpy(pValue, skKeyPressed, skKeyPressed_len);
#else
        *pLen = 1;
        pValue = *pAttr->pValue;
#endif
        break;

      default:
        // Should never get here!
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  return ( status );
}

/*********************************************************************
 * @fn      sk_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 *
 * @return  Success or Failure
 */
static bStatus_t sk_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset )
{
  bStatus_t status = SUCCESS;

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      case SK_KEYPRESSED_UUID:
        HalLedSet( (HAL_LED_1), HAL_LED_MODE_FLASH );
        HalLedSet( (HAL_LED_2), HAL_LED_MODE_TOGGLE );
        // 串口输出            
        HalUARTWrite (HAL_UART_PORT_0, pValue, len);
        break;
      case GATT_CLIENT_CHAR_CFG_UUID:
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        break;
       
      default:
        // Should never get here!
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    status = ATT_ERR_INVALID_HANDLE;
  }

  return ( status );
}

/*********************************************************************
 * @fn          sk_HandleConnStatusCB
 *
 * @brief       Simple Keys Profile link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
static void sk_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{ 
  // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) && 
           ( !linkDB_Up( connHandle ) ) ) )
    { 
      GATTServApp_InitCharCfg( connHandle, skConfig );
    }
  }
}


/*********************************************************************
*********************************************************************/

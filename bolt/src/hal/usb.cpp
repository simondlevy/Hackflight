/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * usb.c
 */

#include <string.h>

#include <stm32fxxx.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "usb.h"

extern "C" {
#include <usbd_usr.h>
#include <usb_conf.h>
#include <usbd_conf.h>
#include <usbd_desc.h>
#include <usb_dcd.h>
#include <usbd_req.h>
}

#define DEVICE_CLASS_CDC                        0x02
#define DEVICE_SUBCLASS_CDC                     0x00

#define USB_DEVICE_DESCRIPTOR_TYPE              0x01
#define USB_CONFIGURATION_DESCRIPTOR_TYPE       0x02
#define USB_STRING_DESCRIPTOR_TYPE              0x03
#define USB_INTERFACE_DESCRIPTOR_TYPE           0x04
#define USB_ENDPOINT_DESCRIPTOR_TYPE            0x05

#define STANDARD_ENDPOINT_DESC_SIZE             0x09

#define CDC_DATA_IN_PACKET_SIZE                CDC_DATA_MAX_PACKET_SIZE

#define CDC_DATA_OUT_PACKET_SIZE               CDC_DATA_MAX_PACKET_SIZE

/*---------------------------------------------------------------------*/
/*  CDC definitions                                                    */
/*---------------------------------------------------------------------*/

/**************************************************/
/* CDC Requests                                   */
/**************************************************/
#define SEND_ENCAPSULATED_COMMAND               0x00
#define GET_ENCAPSULATED_RESPONSE               0x01
#define SET_COMM_FEATURE                        0x02
#define GET_COMM_FEATURE                        0x03
#define CLEAR_COMM_FEATURE                      0x04
#define SET_LINE_CODING                         0x20
#define GET_LINE_CODING                         0x21
#define SET_CONTROL_LINE_STATE                  0x22
#define SEND_BREAK                              0x23
#define NO_CMD                                  0xFF

__attribute__((section(".ccmbss"))) 
  __ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_dev __ALIGN_END;

static bool didInit = false;
static bool doingTransfer = false;
static bool rxStopped = true;
static uint16_t command = 0xFF;

static const auto QUEUE_ITEM_SIZE = sizeof(USBPacket);

uint8_t rxQueueStorage[5 * QUEUE_ITEM_SIZE];
StaticQueue_t rxQueueBuffer;
QueueHandle_t rxQueue;


uint8_t txQueueStorage[1 * QUEUE_ITEM_SIZE];
StaticQueue_t txQueueBuffer;
QueueHandle_t txQueue;

#define BL_STATE_PTR                ((uint64_t *) SRAM2_BASE)  
#define BL_STATE_KEY                (0x5a5) 
#define BL_STATE_KEY_SHIFT          (32)
#define BL_STATE_VALID(reg, addr)   ((uint64_t)(reg) | ((uint64_t)((addr) | BL_STATE_KEY)) << BL_STATE_KEY_SHIFT)


#define USB_CDC_CONFIG_DESC_SIZ     98

#define CF_INTERFACE                0x0
#define VCP_COM_INTERFACE           0x1

/* Endpoints */
#define CF_IN_EP                    0x81  /* EP1 for data IN */
#define CF_OUT_EP                   0x01  /* EP1 for data OUT */

#define VCP_IN_EP                   0x82  /* EP2 for data IN */
#define VCP_OUT_EP                  0x02  /* EP2 for data OUT */
#define VCP_CMD_EP                  0x83  /* EP3 for command */

#define DEVICE_DESCRIPTOR           0x01
#define CONFIGURATION_DESCRIPTOR    0x02
#define STRING_DESCRIPTOR           0x03
#define INTERFACE_DESCRIPTOR        0x04
#define ENDPOINT_DESCRIPTOR         0x05

#define USB_CDC_IDLE                0
#define USB_CDC_BUSY                1
#define USB_CDC_ZLP                 2

/*********************************************
  CDC specific management functions
 *********************************************/
extern uint8_t USBD_DeviceDesc   [USB_SIZ_DEVICE_DESC];

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma data_alignment=4
#endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint8_t usbd_cdc_CfgDesc  [USB_CDC_CONFIG_DESC_SIZ] __ALIGN_END ;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma data_alignment=4
#endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN static __IO uint32_t  usbd_cdc_AltSet  __ALIGN_END = 0;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma data_alignment=4
#endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint8_t USB_Rx_Buffer   [CDC_DATA_MAX_PACKET_SIZE] __ALIGN_END ;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma data_alignment=4
#endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint8_t APP_Rx_Buffer   [APP_RX_DATA_SIZE] __ALIGN_END ;


#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma data_alignment=4
#endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint8_t CmdBuff[CDC_CMD_PACKET_SIZE] __ALIGN_END ;

uint8_t  USB_Tx_State = USB_CDC_IDLE;

static uint32_t cdcCmd = 0xFF;
static uint32_t cdcLen = 0;

__ALIGN_BEGIN uint8_t  usbd_cf_CfgDesc[USB_CDC_CONFIG_DESC_SIZ] __ALIGN_END = {
    /***** Configuration descriptor ******/
    9,                         //bLength
    CONFIGURATION_DESCRIPTOR,  //bDescriptorType
    USB_CDC_CONFIG_DESC_SIZ,   //wTotalLength:no of returned bytes
    0x00,
    3,                         //bNumInterfaces:  3 interfaces (1 for CF, 2 for CDC)
    1,                         //bConfigurationValue
    0,                         //iConfiguration
    0x80,                      //bmAttribute (Bus powered, no remote wakeup)
    50,                        //bMaxPower (100mA, shall be enough)


    /***** Interface 0 descriptor: Crazyflie EPs ******/
    9,                         //bLength
    INTERFACE_DESCRIPTOR,      //bDescriptorType
    CF_INTERFACE,              //bInterfaceNumber
    0,                         //bAlternateSetting
  2,                         //bNumEndpoint (one in, one out)
  0xFF,                      //bInterfaceClass (VENDOR=0xFF)
  0xFF,                      //bInterfaceSubClass (VENDOR=0xFF)
  0,                         //bInterfaceProtocol (None)
  0,                         //iInterface
  /***** Endpoint 1 IN descriptor ******/
  7,                         //bLength
  ENDPOINT_DESCRIPTOR,       //bDescriptorType
  CF_IN_EP,                 //bEndpointAddess (EP1 IN)
  0x02,                      //bmAttributes (Bulk endpoint)
  0x40, 0x00,                //wMaxPacketSize (64 bytes)
  6,                         //bInterval (irrelevant for bulk endpoint)
  /***** Endpoint 1 OUT descriptor ******/
  7,                         //bLength
  ENDPOINT_DESCRIPTOR,       //bDescriptorType
  CF_OUT_EP,                //bEndpointAddess (EP1 OUT)
  0x02,                      //bmAttributes (Bulk endpoint)
  0x40, 0x00,                //wMaxPacketSize (64 bytes)
  6,                         //bInterval (irrelevant for bulk endpoint)


    /*---------------------------------------------------------------------------*/
    /////////////////////////////////////////////////////
    //Add 1 IAD class here // this one is for COM port
    0x08, // bLength: Interface Descriptor size
    0x0B, // bDescriptorType: IAD
    0x01, // bFirstInterface
    0x02, // bInterfaceCount
    0x02, // bFunctionClass: CDC
    0x02, // bFunctionSubClass
    0x01, // bFunctionProtocol
    0x02, // iFunction

    /*Interface Descriptor */
    0x09,   /* bLength: Interface Descriptor size */
    USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: Interface */
    /* Interface descriptor type */
    VCP_COM_INTERFACE,   /* bInterfaceNumber: Number of Interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x01,   /* bNumEndpoints: One endpoints used */
    0x02,   /* bInterfaceClass: Communication Interface Class */
    0x02,   /* bInterfaceSubClass: Abstract Control Model */
    0x01,   /* bInterfaceProtocol: Common AT commands */
    0x01,   /* iInterface: */

    /*Header Functional Descriptor*/
    0x05,   /* bLength: Endpoint Descriptor size */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x00,   /* bDescriptorSubtype: Header Func Desc */
    0x10,   /* bcdCDC: spec release number */
    0x01,

    /*Call Management Functional Descriptor*/
    0x05,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x01,   /* bDescriptorSubtype: Call Management Func Desc */
    0x00,   /* bmCapabilities: D0+D1 */
    0x02,   /* bDataInterface: 2 */

    /*ACM Functional Descriptor*/
    0x04,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
    0x00,   /* bmCapabilities */

    /*Union Functional Descriptor*/
    0x05,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x06,   /* bDescriptorSubtype: Union func desc */
    0x01,   /* bMasterInterface: Communication class interface */
    0x02,   /* bSlaveInterface0: Data Class Interface */

    /*Endpoint 2 Descriptor*/
    0x07,                           /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
    VCP_CMD_EP,                     /* bEndpointAddress */
    0x03,                           /* bmAttributes: Interrupt */
    LOBYTE(CDC_CMD_PACKET_SIZE),     /* wMaxPacketSize: */
    HIBYTE(CDC_CMD_PACKET_SIZE),
#ifdef USE_USB_OTG_HS
    0x10,                           /* bInterval: */
#else
    0xFF,                           /* bInterval: */
#endif /* USE_USB_OTG_HS */
    /*---------------------------------------------------------------------------*/

    /*Data class interface descriptor*/
    0x09,   /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: */
    VCP_COM_INTERFACE + 0x01, /* bInterfaceNumber: Number of Interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x02,   /* bNumEndpoints: Two endpoints used */
    0x0A,   /* bInterfaceClass: CDC */
    0x00,   /* bInterfaceSubClass: */
    0x00,   /* bInterfaceProtocol: */
    0x00,   /* iInterface: */

    /*Endpoint OUT Descriptor*/
    0x07,   /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
    VCP_OUT_EP,                        /* bEndpointAddress */
    0x02,                              /* bmAttributes: Bulk */
    LOBYTE(CDC_DATA_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
    HIBYTE(CDC_DATA_MAX_PACKET_SIZE),
    0x00,                              /* bInterval: ignore for Bulk transfer */

    /*Endpoint IN Descriptor*/
    0x07,   /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
    VCP_IN_EP,                         /* bEndpointAddress */
    0x02,                              /* bmAttributes: Bulk */
    LOBYTE(CDC_DATA_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
    HIBYTE(CDC_DATA_MAX_PACKET_SIZE),
    0x00                               /* bInterval: ignore for Bulk transfer */
};

static uint8_t  usbd_cf_Init        (void  *pdev, uint8_t cfgidx);
static uint8_t  usbd_cf_DeInit      (void  *pdev, uint8_t cfgidx);
static uint8_t  usbd_cf_DataIn      (void *pdev, uint8_t epnum);
static uint8_t  usbd_cf_DataOut     (void *pdev, uint8_t epnum);
static uint8_t  *usbd_cf_GetCfgDesc (uint8_t speed, uint16_t *length);
static uint8_t  usbd_cf_SOF         (void *pdev);
static uint8_t  usbd_cf_Setup       (void *pdev , USB_SETUP_REQ  *req);
static uint8_t  usbd_cdc_EP0_RxReady(void *pdev);

static USBPacket inPacket;
static USBPacket outPacket;

/* CDC interface class callbacks structure */
USBD_Class_cb_TypeDef cf_usb_cb =
{
  usbd_cf_Init,
  usbd_cf_DeInit,
  usbd_cf_Setup,
  NULL,
  usbd_cdc_EP0_RxReady,
  usbd_cf_DataIn,
  usbd_cf_DataOut,
  usbd_cf_SOF,
  NULL,
  NULL,
  usbd_cf_GetCfgDesc,
};

USBD_Usr_cb_TypeDef USR_cb =
{
  USBD_USR_Init,
  USBD_USR_DeviceReset,
  USBD_USR_DeviceConfigured,
  USBD_USR_DeviceSuspended,
  USBD_USR_DeviceResumed,
  USBD_USR_DeviceConnected,
  USBD_USR_DeviceDisconnected,
};

static void enter_bootloader(uint32_t r0, uint32_t bl_addr)
{
    *BL_STATE_PTR = BL_STATE_VALID(r0, bl_addr);
    
    NVIC_SystemReset();
}
static uint8_t usbd_cf_Setup(void *pdev , USB_SETUP_REQ  *req)
{
    if ((req->bmRequest & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_VENDOR) {
        command = req->wIndex;
        if (command == 0x01) {

            if (rxStopped && !xQueueIsQueueFullFromISR(rxQueue))
            {
                DCD_EP_PrepareRx(&USB_OTG_dev,
                        CF_OUT_EP,
                        (uint8_t*)(inPacket.data),
                        USB_RX_TX_PACKET_SIZE);
                rxStopped = false;
            }
        }
        else if (command == 0x02)
        {
            //restart system and transition to DFU bootloader mode
            //enter bootloader specific to STM32f4xx
            enter_bootloader(0, 0x00000000);
        }
    }
    else // VCP_COM_INTERFACE
    {
        switch (req->bmRequest & USB_REQ_TYPE_MASK)
        {
            /* CDC Class Requests -------------------------------*/
            case USB_REQ_TYPE_CLASS :
                /* Check if the request is a data setup packet */
                if (req->wLength)
                {
                    /* Check if the request is Device-to-Host */
                    if (req->bmRequest & 0x80)
                    {
                        /* Get the data to be sent to Host from interface layer */
                        //APP_FOPS.pIf_Ctrl(req->bRequest, CmdBuff, req->wLength);

                        /* Send the data to the host */
                        USBD_CtlSendData ((USB_OTG_CORE_HANDLE*)pdev,
                                CmdBuff,
                                req->wLength);
                    }
                    else /* Host-to-Device request */
                    {
                        /* Set the value of the current command to be processed */
                        cdcCmd = req->bRequest;
                        cdcLen = req->wLength;

                        /* Prepare the reception of the buffer over EP0
                           Next step: the received data will be managed in usbd_cdc_EP0_TxSent()
                           function. */
                        USBD_CtlPrepareRx ((USB_OTG_CORE_HANDLE*)pdev,
                                CmdBuff,
                                req->wLength);
                    }
                }

                return USBD_OK;

            default:
                USBD_CtlError ((USB_OTG_CORE_HANDLE*)pdev, req);
                return USBD_FAIL;

                /* Standard Requests -------------------------------*/
            case USB_REQ_TYPE_STANDARD:
                switch (req->bRequest)
                {
                    case USB_REQ_GET_DESCRIPTOR:
                        USBD_CtlError ((USB_OTG_CORE_HANDLE*)pdev, req);
                        return USBD_FAIL;

                    case USB_REQ_GET_INTERFACE :
                        USBD_CtlSendData ((USB_OTG_CORE_HANDLE*)pdev,
                                (uint8_t *)&usbd_cdc_AltSet,
                                1);
                        break;

                    case USB_REQ_SET_INTERFACE :
                        if ((uint8_t)(req->wValue) < USBD_ITF_MAX_NUM)
                        {
                            usbd_cdc_AltSet = (uint8_t)(req->wValue);
                        }
                        else
                        {
                            /* Call the error management function (command will be nacked */
                            USBD_CtlError ((USB_OTG_CORE_HANDLE*)pdev, req);
                        }
                        break;
                }
        }
    }

    return USBD_OK;
}

static uint8_t  usbd_cf_Init (void  *pdev,
        uint8_t cfgidx)
{
    uint8_t *pbuf;

    /* Open CF EP IN */
    DCD_EP_Open((USB_OTG_CORE_HANDLE*)pdev,
            CF_IN_EP,
            USB_RX_TX_PACKET_SIZE,
            USB_OTG_EP_BULK);

    /* Open CF EP OUT */
    DCD_EP_Open((USB_OTG_CORE_HANDLE*)pdev,
            CF_OUT_EP,
            USB_RX_TX_PACKET_SIZE,
            USB_OTG_EP_BULK);

    /* Prepare Out endpoint to receive next packet */
    DCD_EP_PrepareRx((USB_OTG_CORE_HANDLE*)pdev,
            CF_OUT_EP,
            (uint8_t*)(inPacket.data),
            USB_RX_TX_PACKET_SIZE);
    rxStopped = false;

    /* Open EP IN for VCP*/
    DCD_EP_Open((USB_OTG_CORE_HANDLE*)pdev,
            VCP_IN_EP,
            CDC_DATA_IN_PACKET_SIZE,
            USB_OTG_EP_BULK);

    /* Open EP OUT for VCP*/
    DCD_EP_Open((USB_OTG_CORE_HANDLE*)pdev,
            VCP_OUT_EP,
            CDC_DATA_OUT_PACKET_SIZE,
            USB_OTG_EP_BULK);

    /* Open Command IN EP for VCP*/
    DCD_EP_Open((USB_OTG_CORE_HANDLE*)pdev,
            VCP_CMD_EP,
            CDC_CMD_PACKET_SIZE,
            USB_OTG_EP_INT);

    pbuf = (uint8_t *)USBD_DeviceDesc;
    pbuf[4] = DEVICE_CLASS_CDC;
    pbuf[5] = DEVICE_SUBCLASS_CDC;

    /* Prepare Out endpoint to receive next packet */
    DCD_EP_PrepareRx((USB_OTG_CORE_HANDLE*)pdev,
            VCP_OUT_EP,
            (uint8_t*)(USB_Rx_Buffer),
            CDC_DATA_OUT_PACKET_SIZE);



    return USBD_OK;
}

/**
 * @brief  usbd_cdc_Init
 *         DeInitialize the CDC layer
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t  usbd_cf_DeInit (void  *pdev,
        uint8_t cfgidx)
{
    /* Close CF EP IN */
    DCD_EP_Close((USB_OTG_CORE_HANDLE*)pdev, CF_IN_EP);

    /* Close CF EP OUT */
    DCD_EP_Close((USB_OTG_CORE_HANDLE*)pdev, CF_OUT_EP);

    /* Close VCP EP IN */
    DCD_EP_Close((USB_OTG_CORE_HANDLE*)pdev, VCP_IN_EP);

    /* Close VCP EP OUT */
    DCD_EP_Close((USB_OTG_CORE_HANDLE*)pdev, VCP_OUT_EP);

    /* Close Command IN EP */
    DCD_EP_Close((USB_OTG_CORE_HANDLE*)pdev, VCP_CMD_EP);

    return USBD_OK;
}

/**
 * @brief  usbd_cdc_EP0_RxReady
 *         Data received on control endpoint
 * @param  pdev: device instance
 * @retval status
 */
uint8_t  usbd_cdc_EP0_RxReady (void  *pdev)
{
    if (cdcCmd != NO_CMD)
    {
        /* Process the data */

        /* Reset the command variable to default value */
        cdcCmd = NO_CMD;
    }

    return USBD_OK;
}

/**
 * @brief  usbd_audio_DataIn
 *         Data sent on non-control IN endpoint
 * @param  pdev: device instance
 * @param  epnum: endpoint number
 * @retval status
 */
static uint8_t  usbd_cf_DataIn (void *pdev, uint8_t epnum)
{
    if (epnum == (CF_IN_EP &~ 0x80))
    {
        portBASE_TYPE xTaskWokenByReceive = pdFALSE;

        doingTransfer = false;

        if (xQueueReceiveFromISR(txQueue, &outPacket, &xTaskWokenByReceive) == pdTRUE)
        {
            doingTransfer = true;
            DCD_EP_Tx ((USB_OTG_CORE_HANDLE*)pdev,
                    CF_IN_EP,
                    (uint8_t*)outPacket.data,
                    outPacket.size);
        }

        portYIELD_FROM_ISR(xTaskWokenByReceive);
    }

    return USBD_OK;
}

static uint8_t  usbd_cf_SOF (void *pdev)
{
    portBASE_TYPE xTaskWokenByReceive = pdFALSE;
    if (!doingTransfer) {
        if (xQueueReceiveFromISR(txQueue, &outPacket, &xTaskWokenByReceive) == pdTRUE)
        {
            doingTransfer = true;
            DCD_EP_Tx ((USB_OTG_CORE_HANDLE*)pdev,
                    CF_IN_EP,
                    (uint8_t*)outPacket.data,
                    outPacket.size);
        }
    }
    portYIELD_FROM_ISR(xTaskWokenByReceive);

    return USBD_OK;
}

/**
 * @brief  usbd_cf_DataOut
 *         Data received on non-control Out endpoint
 * @param  pdev: device instance
 * @param  epnum: endpoint number
 * @retval status
 */
static uint8_t  usbd_cf_DataOut (void *pdev, uint8_t epnum)
{
    uint8_t result;
    if (epnum == CF_OUT_EP)
    {
        portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

        /* Get the received data buffer and update the counter */
        inPacket.size = ((USB_OTG_CORE_HANDLE*)pdev)->dev.out_ep[epnum].xfer_count;

        if (xQueueSendFromISR(rxQueue, &inPacket, &xHigherPriorityTaskWoken) == pdTRUE) {
            result = USBD_OK;
        } else {
            result = USBD_BUSY;
        }

        if (!xQueueIsQueueFullFromISR(rxQueue)) {
            /* Prepare Out endpoint to receive next packet */
            DCD_EP_PrepareRx((USB_OTG_CORE_HANDLE*)pdev,
                    CF_OUT_EP,
                    (uint8_t*)(inPacket.data),
                    USB_RX_TX_PACKET_SIZE);
            rxStopped = false;
        } else {
            rxStopped = true;
        }
    }
    else // VCP
    {
        uint16_t USB_Rx_Cnt;

        /* Get the received data buffer and update the counter */
        USB_Rx_Cnt = ((USB_OTG_CORE_HANDLE*)pdev)->dev.out_ep[epnum].xfer_count;

        /* USB data will be immediately processed, this allow next USB traffic being
           NAKed till the end of the application Xfer */

        /* Prepare Out endpoint to receive next packet */
        DCD_EP_PrepareRx((USB_OTG_CORE_HANDLE*)pdev,
                VCP_OUT_EP,
                (uint8_t*)(USB_Rx_Buffer),
                CDC_DATA_OUT_PACKET_SIZE);

        result = USBD_OK;
    }

    return result;
}

/**
 * @brief  USBD_cdc_GetCfgDesc
 *         Return configuration descriptor
 * @param  speed : current device speed
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t  *usbd_cf_GetCfgDesc (uint8_t speed, uint16_t *length)
{
    *length = sizeof (usbd_cf_CfgDesc);
    return usbd_cf_CfgDesc;
}

/**
 * @brief  USBD_USR_Init
 *         Displays the message on LCD for host lib initialization
 * @param  None
 * @retval None
 */
void USBD_USR_Init(void)
{
}

/**
 * @brief  USBD_USR_DeviceReset
 * @param  speed : device speed
 * @retval None
 */
void USBD_USR_DeviceReset(uint8_t speed)
{
}


/**
 * @brief  USBD_USR_DeviceConfigured
 * @param  None
 * @retval Staus
 */
void USBD_USR_DeviceConfigured(void)
{
}

/**
 * @brief  USBD_USR_DeviceSuspended
 * @param  None
 * @retval None
 */
void USBD_USR_DeviceSuspended(void)
{
}


/**
 * @brief  USBD_USR_DeviceResumed
 * @param  None
 * @retval None
 */
void USBD_USR_DeviceResumed(void)
{
}


/**
 * @brief  USBD_USR_DeviceConnected
 * @param  None
 * @retval Staus
 */
void USBD_USR_DeviceConnected(void)
{
}


/**
 * @brief  USBD_USR_DeviceDisonnected
 * @param  None
 * @retval Staus
 */
void USBD_USR_DeviceDisconnected(void)
{
}

void usbInit(void)
{
    USBD_Init(&USB_OTG_dev,
            USB_OTG_FS_CORE_ID,
            &USR_desc,
            &cf_usb_cb,
            &USR_cb);

    rxQueue = xQueueCreateStatic(
            5,
            QUEUE_ITEM_SIZE,
            rxQueueStorage,
            &rxQueueBuffer);

    txQueue = xQueueCreateStatic(
            1,
            QUEUE_ITEM_SIZE,
            txQueueStorage,
            &txQueueBuffer);

    didInit = true;
}

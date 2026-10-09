/**
  ******************************************************************************
  * @file    usbd_pybricks.c
  * @author  MCD Application Team
  * @brief   This file provides the HID core functions.
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  * @verbatim
  *
  *          ===================================================================
  *                                Pybricks Class  Description
  *          ===================================================================
  *
  *
  *
  *
  *
  *
  * @note     In HS mode and when the DMA is used, all variables and data structures
  *           dealing with the DMA during the transaction process should be 32-bit aligned.
  *
  *
  *  @endverbatim
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_ctlreq.h"
#include "usbd_pybricks.h"

#include "../usb_ch9.h"


/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_Pybricks
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_Pybricks_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_Pybricks_Private_Defines
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_Pybricks_Private_Macros
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_Pybricks_Private_FunctionPrototypes
  * @{
  */

static USBD_StatusTypeDef USBD_Pybricks_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static USBD_StatusTypeDef USBD_Pybricks_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static USBD_StatusTypeDef USBD_Pybricks_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static USBD_StatusTypeDef USBD_Pybricks_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static USBD_StatusTypeDef USBD_Pybricks_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static USBD_StatusTypeDef USBD_Pybricks_EP0_RxReady(USBD_HandleTypeDef *pdev);
static uint8_t *USBD_Pybricks_GetCfgDesc(uint16_t *length);

/**
  * @}
  */

/** @defgroup USBD_Pybricks_Private_Variables
  * @{
  */

USBD_ClassTypeDef USBD_Pybricks_ClassDriver =
{
    .Init = USBD_Pybricks_Init,
    .DeInit = USBD_Pybricks_DeInit,
    .Setup = USBD_Pybricks_Setup,
    .EP0_RxReady = USBD_Pybricks_EP0_RxReady,
    .DataIn = USBD_Pybricks_DataIn,
    .DataOut = USBD_Pybricks_DataOut,
    .GetFSConfigDescriptor = USBD_Pybricks_GetCfgDesc,
};

// MS OS 2.0 Descriptor Set — returned for vendor request bRequest=0x20, wIndex=0x0007.
// Assigns WinUSB compatible ID to interface 1 (bulk data, 0xFF/0xC5/0xF6) so Chrome on
// Windows can claimInterface without Zadig.  Size must match wMSOSDescriptorSetTotalLength
// in USBD_BOSDescriptor (usbd_desc.c): 46 bytes = 0x2E.
static uint8_t USBD_MSOS20_DescriptorSet[] = {
    // MS OS 2.0 Descriptor Set Header (10 bytes)
    0x0A, 0x00,              // wLength
    0x00, 0x00,              // wDescriptorType = MS_OS_20_SET_HEADER_DESCRIPTOR
    0x00, 0x00, 0x03, 0x06,  // dwWindowsVersion = 0x06030000 (Windows 8.1+)
    0xB2, 0x00,              // wTotalLength = 178

    // MS OS 2.0 Configuration Subset Header (8 bytes)
    0x08, 0x00,              // wLength
    0x01, 0x00,              // wDescriptorType = MS_OS_20_SUBSET_HEADER_CONFIGURATION
    0x00,                    // bConfigurationValue (0 = configuration 1)
    0x00,                    // bReserved
    0xA8, 0x00,              // wTotalLength = 168

    // MS OS 2.0 Function Subset Header (8 bytes)
    0x08, 0x00,              // wLength
    0x02, 0x00,              // wDescriptorType = MS_OS_20_SUBSET_HEADER_FUNCTION
    0x00,                    // bFirstInterface = 0 — matches IAD.bFirstInterface
    0x00,                    // bReserved
    0xA0, 0x00,              // wSubsetLength = 160

    // MS OS 2.0 Compatible ID Descriptor (20 bytes)
    0x14, 0x00,              // wLength
    0x03, 0x00,              // wDescriptorType = MS_OS_20_FEATURE_COMPATIBLE_ID
    'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,  // CompatibleID = "WINUSB"
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // SubCompatibleID

    // MS OS 2.0 Registry Property Descriptor (132 bytes)
    // Causes Windows to write DeviceInterfaceGUIDs to the registry on driver install,
    // so Chrome can find the WinUSB device interface path for claimInterface.
    0x84, 0x00,              // wLength = 132
    0x04, 0x00,              // wDescriptorType = MS_OS_20_FEATURE_REG_PROPERTY
    0x07, 0x00,              // wPropertyDataType = REG_MULTI_SZ
    0x2A, 0x00,              // wPropertyNameLength = 42 (20 chars × 2 + 2 null)
    // PropertyName = "DeviceInterfaceGUIDs" (UTF-16LE, 42 bytes incl. null terminator)
    'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00, 
    'I', 0x00, 'n', 0x00, 't', 0x00, 'e', 0x00, 'r', 0x00, 'f', 0x00, 
    'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00, 
    'D', 0x00, 's', 0x00, 
    0x00, 0x00,              // null terminator of PropertyName
    0x50, 0x00,              // wPropertyDataLength = 80
    // PropertyData = "{82ED08B2-372A-4C39-A5BD-5B0DEACF04E5}\0\0" (UTF-16LE, 80 bytes)
    '{', 0x00, '8', 0x00, '2', 0x00, 'E', 0x00, 'D', 0x00, '0', 0x00, '8', 0x00, 'B', 0x00, 
    '2', 0x00, '-', 0x00, '3', 0x00, '7', 0x00, '2', 0x00, 'A', 0x00, '-', 0x00, '4', 0x00, 
    'C', 0x00, '3', 0x00, '9', 0x00, '-', 0x00, 'A', 0x00, '5', 0x00, 'B', 0x00, 'D', 0x00, 
    '-', 0x00, '5', 0x00, 'B', 0x00, '0', 0x00, 'D', 0x00, 'E', 0x00, 'A', 0x00, 'C', 0x00, 
    'F', 0x00, '0', 0x00, '4', 0x00, 'E', 0x00, '5', 0x00, '}', 0x00, 
    0x00, 0x00,              // null terminator (end of GUID string)
    0x00, 0x00,              // REG_MULTI_SZ list terminator
};

/* USB Pybricks device Configuration Descriptor */
typedef struct PBDRV_PACKED {
    pbdrv_usb_conf_desc_t conf_desc;
    pbdrv_usb_iad_desc_t iad;
    pbdrv_usb_iface_desc_t comm_iface;
    pbdrv_usb_cdc_header_desc_t cdc_header;
    pbdrv_usb_cdc_call_mgmt_desc_t cdc_call_mgmt;
    pbdrv_usb_cdc_acm_desc_t cdc_acm;
    pbdrv_usb_cdc_union_desc_t cdc_union;
    pbdrv_usb_ep_desc_t cmd_ep;
    pbdrv_usb_iface_desc_t data_iface;
    pbdrv_usb_ep_desc_t ep_out;
    pbdrv_usb_ep_desc_t ep_in;
} pbdrv_usb_stm32_conf_t;
PBDRV_USB_TYPE_PUNNING_HELPER(pbdrv_usb_stm32_conf);

static pbdrv_usb_stm32_conf_union_t USBD_Pybricks_CfgDesc = {
    .s = {
        .conf_desc = {
            .bLength = sizeof(pbdrv_usb_conf_desc_t),
            .bDescriptorType = DESC_TYPE_CONFIGURATION,
            .wTotalLength = sizeof(pbdrv_usb_stm32_conf_t),
            .bNumInterfaces = 2,
            .bConfigurationValue = 1,
            .iConfiguration = 0,
            .bmAttributes = USB_CONF_DESC_BM_ATTR_MUST_BE_SET,
            .bMaxPower = 250,   /* 500mA (number of 2mA units) */
        },
        /* Interface Association: vendor-specific to prevent Windows/ChromeOS
         * from loading usbser.sys / CDC driver, which would block WebUSB. */
        .iad = {
            .bLength = sizeof(pbdrv_usb_iad_desc_t),
            .bDescriptorType = DESC_TYPE_INTERFACE_ASSOCIATION,
            .bFirstInterface = 0,
            .bInterfaceCount = 2,
            .bFunctionClass = 0xFF,
            .bFunctionSubClass = 0x00,
            .bFunctionProtocol = 0x00,
            .iFunction = 0,
        },
        /* Communication interface — vendor-specific (0xFF) so no OS driver claims it. */
        .comm_iface = {
            .bLength = sizeof(pbdrv_usb_iface_desc_t),
            .bDescriptorType = DESC_TYPE_INTERFACE,
            .bInterfaceNumber = 0,
            .bAlternateSetting = 0,
            .bNumEndpoints = 1,
            .bInterfaceClass = 0xFF,
            .bInterfaceSubClass = 0x00,
            .bInterfaceProtocol = 0x00,
            .iInterface = 0,
        },
        .cdc_header = {
            .bFunctionLength = sizeof(pbdrv_usb_cdc_header_desc_t),
            .bDescriptorType = USB_CDC_CS_INTERFACE,
            .bDescriptorSubtype = USB_CDC_FUNC_SUBTYPE_HEADER,
            .bcdCDC = 0x0110,
        },
        .cdc_call_mgmt = {
            .bFunctionLength = sizeof(pbdrv_usb_cdc_call_mgmt_desc_t),
            .bDescriptorType = USB_CDC_CS_INTERFACE,
            .bDescriptorSubtype = USB_CDC_FUNC_SUBTYPE_CALL_MGMT,
            .bmCapabilities = 0x00,
            .bDataInterface = 1,
        },
        .cdc_acm = {
            .bFunctionLength = sizeof(pbdrv_usb_cdc_acm_desc_t),
            .bDescriptorType = USB_CDC_CS_INTERFACE,
            .bDescriptorSubtype = USB_CDC_FUNC_SUBTYPE_ACM,
            .bmCapabilities = 0x02,
        },
        .cdc_union = {
            .bFunctionLength = sizeof(pbdrv_usb_cdc_union_desc_t),
            .bDescriptorType = USB_CDC_CS_INTERFACE,
            .bDescriptorSubtype = USB_CDC_FUNC_SUBTYPE_UNION,
            .bControlInterface = 0,
            .bSubordinateInterface0 = 1,
        },
        .cmd_ep = {
            .bLength = sizeof(pbdrv_usb_ep_desc_t),
            .bDescriptorType = DESC_TYPE_ENDPOINT,
            .bEndpointAddress = USBD_PYBRICKS_CMD_EP,
            .bmAttributes = PBDRV_USB_EP_TYPE_INTR,
            .wMaxPacketSize = USBD_PYBRICKS_CMD_PACKET_SIZE,
            .bInterval = 16,
        },
        /* Data interface — vendor-specific (0xFF/0xC5/0xF6) so Windows/ChromeOS
         * do not load usbser.sys and WebUSB can claim the interface freely.
         * 0xF6 distinguishes new hub (H562) from old hub (Pybricks 3.x: 0xF5). */
        .data_iface = {
            .bLength = sizeof(pbdrv_usb_iface_desc_t),
            .bDescriptorType = DESC_TYPE_INTERFACE,
            .bInterfaceNumber = 1,
            .bAlternateSetting = 0,
            .bNumEndpoints = 2,
            .bInterfaceClass = 0xFF,
            .bInterfaceSubClass = 0xC5,
            .bInterfaceProtocol = 0xF6,
            .iInterface = 0,
        },
        .ep_out = {
            .bLength = sizeof(pbdrv_usb_ep_desc_t),
            .bDescriptorType = DESC_TYPE_ENDPOINT,
            .bEndpointAddress = USBD_PYBRICKS_OUT_EP,
            .bmAttributes = PBDRV_USB_EP_TYPE_BULK,
            .wMaxPacketSize = USBD_PYBRICKS_MAX_PACKET_SIZE,
            .bInterval = 0,     /* ignore for Bulk transfer */
        },
        .ep_in = {
            .bLength = sizeof(pbdrv_usb_ep_desc_t),
            .bDescriptorType = DESC_TYPE_ENDPOINT,
            .bEndpointAddress = USBD_PYBRICKS_IN_EP,
            .bmAttributes = PBDRV_USB_EP_TYPE_BULK,
            .wMaxPacketSize = USBD_PYBRICKS_MAX_PACKET_SIZE,
            .bInterval = 0,     /* ignore for Bulk transfer */
        },
    }
};

/**
  * @}
  */

/** @defgroup USBD_Pybricks_Private_Functions
  * @{
  */

/**
  * @brief  USBD_Pybricks_Init
  *         Initialize the Pybricks interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static USBD_StatusTypeDef USBD_Pybricks_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
    UNUSED(cfgidx);
    static USBD_Pybricks_HandleTypeDef hPybricks;

    pdev->pClassData = &hPybricks;

    /* Default line coding reported to the host: 115200 baud, 1 stop bit, no
     * parity, 8 data bits. This is inert: USB does not transfer data at this
     * rate (it is not a real UART). CDC ACM just requires valid line coding to
     * be stored and echoed back on GET_LINE_CODING. */
    hPybricks.LineCoding[0] = 0x00;
    hPybricks.LineCoding[1] = 0xC2;
    hPybricks.LineCoding[2] = 0x01;
    hPybricks.LineCoding[3] = 0x00;
    hPybricks.LineCoding[4] = 0x00;
    hPybricks.LineCoding[5] = 0x00;
    hPybricks.LineCoding[6] = 0x08;
    hPybricks.CmdOpCode = 0xFFU;

    (void)USBD_LL_OpenEP(pdev, USBD_PYBRICKS_IN_EP, USBD_EP_TYPE_BULK, USBD_PYBRICKS_MAX_PACKET_SIZE);
    pdev->ep_in[USBD_PYBRICKS_IN_EP & 0xFU].is_used = 1U;

    (void)USBD_LL_OpenEP(pdev, USBD_PYBRICKS_OUT_EP, USBD_EP_TYPE_BULK, USBD_PYBRICKS_MAX_PACKET_SIZE);
    pdev->ep_out[USBD_PYBRICKS_OUT_EP & 0xFU].is_used = 1U;

    (void)USBD_LL_OpenEP(pdev, USBD_PYBRICKS_CMD_EP, USBD_EP_TYPE_INTR, USBD_PYBRICKS_CMD_PACKET_SIZE);
    pdev->ep_in[USBD_PYBRICKS_CMD_EP & 0xFU].is_used = 1U;

    /* Init  physical Interface components */
    ((USBD_Pybricks_ItfTypeDef *)pdev->pUserData[pdev->classId])->Init();

    (void)USBD_LL_PrepareReceive(pdev, USBD_PYBRICKS_OUT_EP, hPybricks.RxBuffer, USBD_PYBRICKS_MAX_PACKET_SIZE);

    return USBD_OK;
}

/**
  * @brief  USBD_Pybricks_DeInit
  *         DeInitialize the Pybricks layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static USBD_StatusTypeDef USBD_Pybricks_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
    UNUSED(cfgidx);

    /* Close EP IN */
    (void)USBD_LL_CloseEP(pdev, USBD_PYBRICKS_IN_EP);
    pdev->ep_in[USBD_PYBRICKS_IN_EP & 0xFU].is_used = 0U;

    /* Close EP OUT */
    (void)USBD_LL_CloseEP(pdev, USBD_PYBRICKS_OUT_EP);
    pdev->ep_out[USBD_PYBRICKS_OUT_EP & 0xFU].is_used = 0U;

    /* Close command EP */
    (void)USBD_LL_CloseEP(pdev, USBD_PYBRICKS_CMD_EP);
    pdev->ep_in[USBD_PYBRICKS_CMD_EP & 0xFU].is_used = 0U;

    /* DeInit  physical Interface components */
    if (pdev->pClassData != NULL) {
        ((USBD_Pybricks_ItfTypeDef *)pdev->pUserData[pdev->classId])->DeInit();
        pdev->pClassData = NULL;
    }

    return USBD_OK;
}

/**
  * @brief  USBD_Pybricks_Setup
  *         Handle the Pybricks specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static USBD_StatusTypeDef USBD_Pybricks_Setup(USBD_HandleTypeDef *pdev,
    USBD_SetupReqTypedef *req) {
    USBD_Pybricks_HandleTypeDef *hPybricks = pdev->pClassData;
    uint8_t ifalt = 0U;
    uint16_t status_info = 0U;
    USBD_StatusTypeDef ret = USBD_OK;

    switch (req->bmRequest & USB_REQ_TYPE_MASK)
    {
        case USB_REQ_TYPE_CLASS:
            if (hPybricks == NULL) {
                USBD_CtlError(pdev, req);
                ret = USBD_FAIL;
                break;
            }
            switch (req->bRequest)
            {
                case USB_CDC_REQ_SET_LINE_CODING:
                    /* Receive the line coding into the handle. The value is
                     * accepted but not acted on (the link is not a real UART). */
                    if (req->wLength == USB_CDC_LINE_CODING_SIZE) {
                        hPybricks->CmdOpCode = (uint8_t)req->bRequest;
                        hPybricks->CmdLength = USB_CDC_LINE_CODING_SIZE;
                        (void)USBD_CtlPrepareRx(pdev, hPybricks->LineCoding, USB_CDC_LINE_CODING_SIZE);
                    } else {
                        USBD_CtlError(pdev, req);
                        ret = USBD_FAIL;
                    }
                    break;

                case USB_CDC_REQ_GET_LINE_CODING:
                    (void)USBD_CtlSendData(pdev, hPybricks->LineCoding,
                        MIN(USB_CDC_LINE_CODING_SIZE, req->wLength));
                    break;

                case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
                    /* The DTR bit indicates whether a host application has
                     * opened the port. This is the USB analog of a BLE host
                     * subscribing to notifications. */
                    ((USBD_Pybricks_ItfTypeDef *)pdev->pUserData[pdev->classId])->SetControlLineState(
                        (req->wValue & USB_CDC_CONTROL_LINE_STATE_DTR) != 0U);
                    break;

                default:
                    USBD_CtlError(pdev, req);
                    ret = USBD_FAIL;
                    break;
            }
            break;

        case USB_REQ_TYPE_STANDARD:
            switch (req->bRequest)
            {
                case USB_REQ_GET_STATUS:
                    if (pdev->dev_state == USBD_STATE_CONFIGURED) {
                        (void)USBD_CtlSendData(pdev, (uint8_t *)&status_info, 2U);
                    } else {
                        USBD_CtlError(pdev, req);
                        ret = USBD_FAIL;
                    }
                    break;

                case USB_REQ_GET_INTERFACE:
                    if (pdev->dev_state == USBD_STATE_CONFIGURED) {
                        (void)USBD_CtlSendData(pdev, &ifalt, 1U);
                    } else {
                        USBD_CtlError(pdev, req);
                        ret = USBD_FAIL;
                    }
                    break;

                case USB_REQ_SET_INTERFACE:
                    if (pdev->dev_state != USBD_STATE_CONFIGURED) {
                        USBD_CtlError(pdev, req);
                        ret = USBD_FAIL;
                    }
                    break;

                case USB_REQ_CLEAR_FEATURE:
                    break;

                default:
                    USBD_CtlError(pdev, req);
                    ret = USBD_FAIL;
                    break;
            }
            break;

        case USB_REQ_TYPE_VENDOR:
            // MS OS 2.0 Descriptor Set request: bRequest=0x20, wIndex=0x0007
            if ((req->bmRequest & 0x80U) != 0U &&
                req->bRequest == 0x20U &&
                req->wIndex == 0x0007U) {
                uint16_t len = MIN((uint16_t)sizeof(USBD_MSOS20_DescriptorSet), req->wLength);
                (void)USBD_CtlSendData(pdev, USBD_MSOS20_DescriptorSet, len);
            } else {
                USBD_CtlError(pdev, req);
                ret = USBD_FAIL;
            }
            break;

        default:
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
            break;
    }

    return ret;
}

/**
  * @brief  USBD_Pybricks_GetCfgDesc
  *         return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_Pybricks_GetCfgDesc(uint16_t *length) {
    *length = (uint16_t)sizeof(USBD_Pybricks_CfgDesc.s);
    return (uint8_t *)&USBD_Pybricks_CfgDesc;
}

/**
  * @brief  USBD_Pybricks_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static USBD_StatusTypeDef USBD_Pybricks_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum) {
    USBD_Pybricks_HandleTypeDef *hPybricks = pdev->pClassData;
    PCD_HandleTypeDef *hpcd = pdev->pData;

    if (hPybricks == NULL) {
        return USBD_FAIL;
    }

    if ((pdev->ep_in[epnum].total_length > 0U) &&
        ((pdev->ep_in[epnum].total_length % hpcd->IN_ep[epnum].maxpacket) == 0U)) {
        /* Update the packet total length */
        pdev->ep_in[epnum].total_length = 0U;

        /* Send ZLP */
        (void)USBD_LL_Transmit(pdev, epnum, NULL, 0U);
    } else {
        ((USBD_Pybricks_ItfTypeDef *)pdev->pUserData[pdev->classId])->TransmitCplt(hPybricks->TxBuffer, hPybricks->TxLength, epnum);
    }

    return USBD_OK;
}

/**
  * @brief  USBD_Pybricks_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static USBD_StatusTypeDef USBD_Pybricks_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum) {
    USBD_Pybricks_HandleTypeDef *hPybricks = pdev->pClassData;

    if (hPybricks == NULL) {
        return USBD_FAIL;
    }

    /* Get the received data length */
    hPybricks->RxLength = USBD_LL_GetRxDataSize(pdev, epnum);

    /* USB data will be immediately processed, this allow next USB traffic being
    NAKed till the end of the application Xfer */

    ((USBD_Pybricks_ItfTypeDef *)pdev->pUserData[pdev->classId])->Receive(hPybricks->RxBuffer, hPybricks->RxLength);

    return USBD_OK;
}

/**
  * @brief  USBD_Pybricks_EP0_RxReady
  *         Handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static USBD_StatusTypeDef USBD_Pybricks_EP0_RxReady(USBD_HandleTypeDef *pdev) {
    USBD_Pybricks_HandleTypeDef *hPybricks = pdev->pClassData;

    if (hPybricks != NULL && hPybricks->CmdOpCode != 0xFFU) {
        /* SET_LINE_CODING data has been received into hPybricks->LineCoding.
         * Nothing to do; the value is stored for GET_LINE_CODING. */
        hPybricks->CmdOpCode = 0xFFU;
    }

    return USBD_OK;
}

/**
* @brief  USBD_Pybricks_RegisterInterface
  * @param  pdev: device instance
  * @param  fops: CD  Interface callback
  * @retval status
  */
USBD_StatusTypeDef USBD_Pybricks_RegisterInterface(USBD_HandleTypeDef *pdev, USBD_Pybricks_ItfTypeDef *fops) {
    if (fops == NULL) {
        return USBD_FAIL;
    }

    pdev->pUserData[pdev->classId] = fops;

    return USBD_OK;
}

/**
  * @brief  USBD_Pybricks_SetRxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Rx Buffer
  * @retval status
  */
USBD_StatusTypeDef USBD_Pybricks_SetRxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff) {
    USBD_Pybricks_HandleTypeDef *hPybricks = pdev->pClassData;

    if (hPybricks == NULL) {
        return USBD_FAIL;
    }

    hPybricks->RxBuffer = pbuff;

    return USBD_OK;
}

/**
  * @brief  USBD_Pybricks_TransmitPacket
  *         Transmit packet on IN endpoint
  * @param  pdev: device instance
  * @retval status
  */
USBD_StatusTypeDef USBD_Pybricks_TransmitPacket(USBD_HandleTypeDef *pdev, uint8_t *pbuf, uint32_t length) {
    USBD_Pybricks_HandleTypeDef *hPybricks = pdev->pClassData;

    if (hPybricks == NULL) {
        return USBD_FAIL;
    }

    hPybricks->TxBuffer = pbuf;
    hPybricks->TxLength = length;

    /* Update the packet total length */
    pdev->ep_in[USBD_PYBRICKS_IN_EP & 0xFU].total_length = hPybricks->TxLength;

    /* Transmit next packet */
    (void)USBD_LL_Transmit(pdev, USBD_PYBRICKS_IN_EP, hPybricks->TxBuffer, hPybricks->TxLength);

    return USBD_OK;
}


/**
  * @brief  USBD_Pybricks_ReceivePacket
  *         prepare OUT Endpoint for reception
  * @param  pdev: device instance
  * @retval status
  */
USBD_StatusTypeDef USBD_Pybricks_ReceivePacket(USBD_HandleTypeDef *pdev) {
    USBD_Pybricks_HandleTypeDef *hPybricks = pdev->pClassData;

    if (hPybricks == NULL) {
        return USBD_FAIL;
    }

    /* Prepare Out endpoint to receive next packet */
    (void)USBD_LL_PrepareReceive(pdev, USBD_PYBRICKS_OUT_EP, hPybricks->RxBuffer, USBD_PYBRICKS_MAX_PACKET_SIZE);

    return USBD_OK;
}

/**
  * @}
  */


/**
  * @}
  */


/**
  * @}
  */

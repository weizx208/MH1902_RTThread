/************************ (C) COPYRIGHT Megahuntmicro *************************
 * File Name            : usbd_hid_core.c
 * Author               : Megahuntmicro
 * Version              : V1.0.0
 * Date                 : 21-October-2014
 * Description          : This file provides the HID core functions
 *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "usbd_hid_core.h"
#include "usbd_desc.h"
#include "usbd_req.h"


/** @defgroup USBD_HID_Private_FunctionPrototypes
  * @{
  */
static usbd_hid_recv_cb_t usbd_hid_recv_cb;
	
static uint8_t  USBD_HID_Init (void  *pdev, uint8_t cfgidx);

static uint8_t  USBD_HID_DeInit (void  *pdev, uint8_t cfgidx);

static uint8_t  USBD_HID_Setup (void  *pdev, USB_SETUP_REQ *req);

static uint8_t  *USBD_HID_GetCfgDesc (uint8_t speed, uint16_t *length);

static uint8_t  USBD_HID_DataIn (void  *pdev, uint8_t epnum);

#ifdef HID_SUPPORT_IN_OUT 
static uint8_t  USBD_HID_DataOut (void  *pdev, uint8_t epnum);
#endif
/**
  * @}
  */ 

/** @defgroup USBD_HID_Private_Variables
  * @{
  */ 
USBD_Class_cb_TypeDef  USBD_HID_cb = 
{
    USBD_HID_Init,
    USBD_HID_DeInit,
    USBD_HID_Setup,
    NULL, /*EP0_TxSent*/  
    NULL, /*EP0_RxReady*/
    USBD_HID_DataIn, /*DataIn*/
#ifndef HID_SUPPORT_IN_OUT
    NULL,
#else   
    USBD_HID_DataOut, /*DataOut*/
#endif    
    NULL, /*SOF */
    NULL,
    NULL,      
    USBD_HID_GetCfgDesc,
};

#ifdef HID_SUPPORT_IN_OUT
__ALIGN_BEGIN uint8_t USBD_HID_RxBuf[HID_OUT_PACKET] __ALIGN_END = {0};
#endif

__ALIGN_BEGIN static uint32_t  USBD_HID_AltSet  __ALIGN_END = 0;
    
__ALIGN_BEGIN static uint32_t  USBD_HID_Protocol  __ALIGN_END = 0;
 
__ALIGN_BEGIN static uint32_t  USBD_HID_IdleState __ALIGN_END = 0;

/* USB HID device Configuration Descriptor */
#ifndef HID_SUPPORT_IN_OUT 
__ALIGN_BEGIN static uint8_t USBD_HID_CfgDesc[USB_HID_CONFIG_DESC_SIZ] __ALIGN_END =
{
    0x09, /* bLength: Configuration Descriptor size */
    USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType: Configuration */
    USB_HID_CONFIG_DESC_SIZ,
    /* wTotalLength: Bytes returned */
    0x00,
    0x01,         /*bNumInterfaces: 1 interface*/
    0x01,         /*bConfigurationValue: Configuration value*/
    0x00,         /*iConfiguration: Index of string descriptor describing
    the configuration*/
    0xE0,         /*bmAttributes: bus powered and Support Remote Wake-up */
    0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/

    /************** Descriptor of Joystick Mouse interface ****************/
    /* 09 */
    0x09,         /*bLength: Interface Descriptor size*/
    USB_INTERFACE_DESCRIPTOR_TYPE,/*bDescriptorType: Interface descriptor type*/
    0x00,         /*bInterfaceNumber: Number of Interface*/
    0x00,         /*bAlternateSetting: Alternate setting*/
    0x01,         /*bNumEndpoints*/
    0x03,         /*bInterfaceClass: HID*/
    0x01,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
    0x02,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
    0,            /*iInterface: Index of string descriptor*/
    /******************** Descriptor of Joystick Mouse HID ********************/
    /* 18 */
    0x09,         /*bLength: HID Descriptor size*/
    HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
    0x11,         /*bcdHID: HID Class Spec release number*/
    0x01,
    0x00,         /*bCountryCode: Hardware target country*/
    0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
    0x22,         /*bDescriptorType*/
    HID_MOUSE_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
    0x00,
    /******************** Descriptor of Mouse endpoint ********************/
    /* 27 */
    0x07,          /*bLength: Endpoint Descriptor size*/
    USB_ENDPOINT_DESCRIPTOR_TYPE, /*bDescriptorType:*/

    HID_IN_EP,     /*bEndpointAddress: Endpoint Address (IN)*/
    0x03,          /*bmAttributes: Interrupt endpoint*/
    HID_IN_PACKET, /*wMaxPacketSize: 4 Byte max */
    0x00,
    0x0A,          /*bInterval: Polling Interval (10 ms)*/
    /* 34 */
} ;
#else
__ALIGN_BEGIN static uint8_t USBD_HID_CfgDesc[USB_HID_CONFIG_DESC_SIZ] __ALIGN_END =
{
    0x09,                               /*  bLength = 9                     */
    USB_CONFIGURATION_DESCRIPTOR_TYPE,  /*  bDescriptorType = Config (2)    */
    USB_HID_CONFIG_DESC_SIZ,            /*  wTotalLength(L/H)               */
    0x00,
    0x01,                               /*  bNumInterfaces                  */
    0x01,           										/*  bConfigValue                    */
    0x00,                               /*  iConfiguration                  */
    0xc0,                               /*  bmAttributes (self-powered, no remote wakeup) */
    0xFA,                               /*  MaxPower is 2ma (units are 2ma/bit) */
    
    /* 09 */    
    0x09,                               /*  bLength                          */
    USB_INTERFACE_DESCRIPTOR_TYPE,      /*  bDescriptorType = Interface (4)  */
    0x00,                               /*  bInterfaceNumber                 */
    0x00,                               /*  bAlternateSetting                */
    0x02,                               /*  bNumEndpoints (IN and OUT)      */
    0X03,                               /*  bInterfaceClass = HID            */
    0x00,                               /*  bInterfaceSubClass : 1=BOOT, 0=no boot  */
    0x00,                               /*  nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse  */
    0x00,                               /*  iInterface */
    
    /* 18 */
    0x09,                               /*  bFunctionalLength                */
    HID_DESCRIPTOR_TYPE,                /*  bDescriptorType = HID            */
    0x10,                               /*  bcdHID Rev 1.1                   */
    0x01,
    0x00,                               /*  bCountryCode                     */
    0x01,                               /*  bNumDescriptors                  */
    0x22,                               /*  bDescriptorType = Report         */
    LOBYTE(HID_MOUSE_REPORT_DESC_SIZE), /*  wDescriptorLength                */
    HIBYTE(HID_MOUSE_REPORT_DESC_SIZE),
    
    /* 27 */    
    0x07,                               /*  bLength                         */
    USB_ENDPOINT_DESCRIPTOR_TYPE,       /*  bDescriptorType (Endpoint)      */
    HID_IN_EP,                          /*  bEndpointAddress                */
    0X03,                               /*  bmAttributes (interrupt)        */
    HID_IN_PACKET,                      /*  wMaxPacketSize                  */
    0x00,
    0x01,                               /*  bInterval (N/A)                 */
    
    /* 34 */    
    0x07,                               /*  bLength                         */
    USB_ENDPOINT_DESCRIPTOR_TYPE,       /*  bDescriptorType (Endpoint)      */
    HID_OUT_EP,                         /*  bEndpointAddress                */
    0X03,                               /*  bmAttributes (interrupt)        */
    HID_OUT_PACKET,                     /*  wMaxPacketSize                  */
    0x00,
    0x01,                               /*  bInterval (N/A)                 */
    /* 41 */
} ;
#endif

#ifndef HID_SUPPORT_IN_OUT 
__ALIGN_BEGIN static uint8_t HID_MOUSE_ReportDesc[HID_MOUSE_REPORT_DESC_SIZE] __ALIGN_END =
{
    0x05,   0x01,
    0x09,   0x02,
    0xA1,   0x01,
    0x09,   0x01,

    0xA1,   0x00,
    0x05,   0x09,
    0x19,   0x01,
    0x29,   0x03,

    0x15,   0x00,
    0x25,   0x01,
    0x95,   0x03,
    0x75,   0x01,

    0x81,   0x02,
    0x95,   0x01,
    0x75,   0x05,
    0x81,   0x01,

    0x05,   0x01,
    0x09,   0x30,
    0x09,   0x31,
    0x09,   0x38,

    0x15,   0x81,
    0x25,   0x7F,
    0x75,   0x08,
    0x95,   0x03,

    0x81,   0x06,
    0xC0,   0x09,
    0x3c,   0x05,
    0xff,   0x09,

    0x01,   0x15,
    0x00,   0x25,
    0x01,   0x75,
    0x01,   0x95,

    0x02,   0xb1,
    0x22,   0x75,
    0x06,   0x95,
    0x01,   0xb1,

    0x01,   0xc0
};
#else
__ALIGN_BEGIN static uint8_t HID_MOUSE_ReportDesc[HID_MOUSE_REPORT_DESC_SIZE] __ALIGN_END =
{
    0x05, 0x01,     // USAGE_PAGE (Generic Desktop)
    0x09, 0x00,     // USAGE (0)
    0xa1, 0x01,     // COLLECTION (Application)
    0x15, 0x00,     //     LOGICAL_MINIMUM (0)
    0x25, 0xff,     //     LOGICAL_MAXIMUM (255)
    0x19, 0x01,     //     USAGE_MINIMUM (1)
    0x29, 0x08,     //     USAGE_MAXIMUM (8)
    0x95, 0x40,     //     REPORT_COUNT (64)
    0x75, 0x08,     //     REPORT_SIZE (8)
    0x81, 0x02,     //     INPUT (Data,Var,Abs)
    0x19, 0x01,     //     USAGE_MINIMUM (1)
    0x29, 0x08,     //     USAGE_MAXIMUM (8)
    0x91, 0x02,     //   OUTPUT (Data,Var,Abs)
    0xc0            // END_COLLECTION
}; 
#endif

/**
  * @}
  */ 

/** @defgroup USBD_HID_Private_Functions
  * @{
  */ 

/**
  * @brief  USBD_HID_Init
  *         Initialize the HID interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_HID_Init (void  *pdev, uint8_t cfgidx)
{
    /* Open EP IN */
    DCD_EP_Open(pdev,
                HID_IN_EP,
                HID_IN_PACKET,
                USB_OTG_EP_INT);

    /* Open EP OUT */
    DCD_EP_Open(pdev,
                HID_OUT_EP,
                HID_OUT_PACKET,
                USB_OTG_EP_INT);

#ifdef HID_SUPPORT_IN_OUT  
    DCD_EP_PrepareRx (pdev,
                      HID_OUT_EP,
                      USBD_HID_RxBuf,
                      HID_OUT_PACKET);
#endif

    return USBD_OK;
}

/**
  * @brief  USBD_HID_Init
  *         DeInitialize the HID layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_HID_DeInit (void  *pdev, uint8_t cfgidx)
{
	/* Close HID EPs */
	DCD_EP_Close (pdev , HID_IN_EP);
	DCD_EP_Close (pdev , HID_OUT_EP);

	return USBD_OK;
}

/**
  * @brief  USBD_HID_Setup
  *         Handle the HID specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_HID_Setup (void  *pdev, USB_SETUP_REQ *req)
{
	uint16_t len = 0;
	uint8_t  *pbuf = NULL;

	switch (req->bmRequest & USB_REQ_TYPE_MASK)
	{
		case USB_REQ_TYPE_CLASS :  
		switch (req->bRequest)
		{
			case HID_REQ_SET_PROTOCOL:
				USBD_HID_Protocol = (uint8_t)(req->wValue);
			break;
	  
			case HID_REQ_GET_PROTOCOL:
				USBD_CtlSendData (pdev, 
								 (uint8_t *)&USBD_HID_Protocol,
								 1);    
			break;
	  
			case HID_REQ_SET_IDLE:
				USBD_HID_IdleState = (uint8_t)(req->wValue >> 8);
			break;
	  
			case HID_REQ_GET_IDLE:
				USBD_CtlSendData (pdev, 
								 (uint8_t *)&USBD_HID_IdleState,
								 1);        
			break;      
	  
			default:
				USBD_CtlError (pdev, req);
			return USBD_FAIL; 
		}
		break;

		case USB_REQ_TYPE_STANDARD:
		switch (req->bRequest)
		{
			case USB_REQ_GET_DESCRIPTOR: 
				if( req->wValue >> 8 == HID_REPORT_DESC)
				{
					len = MIN(HID_MOUSE_REPORT_DESC_SIZE , req->wLength);
					pbuf = HID_MOUSE_ReportDesc;
				}
				else if( req->wValue >> 8 == HID_DESCRIPTOR_TYPE)
				{
					pbuf = USBD_HID_CfgDesc + 0x12;
					len = MIN(USB_HID_DESC_SIZ , req->wLength);
				}
	  
				USBD_CtlSendData (pdev, 
								  pbuf,
								  len);
	  
			break;
	  
			case USB_REQ_GET_INTERFACE :
				USBD_CtlSendData (pdev,
								 (uint8_t *)&USBD_HID_AltSet,
								  1);
			break;
	  
			case USB_REQ_SET_INTERFACE :
				USBD_HID_AltSet = (uint8_t)(req->wValue);
			break;
		}
	}
	return USBD_OK;
}

/**
  * @brief  USBD_HID_SendReport 
  *         Send HID Report
  * @param  pdev: device instance
  * @param  buff: pointer to report
  * @retval status
  */
uint8_t USBD_HID_SendReport     (USB_OTG_CORE_HANDLE  *pdev, 
                                 uint8_t *report,
                                 uint16_t len)
{
	if (pdev->dev.device_status == USB_OTG_CONFIGURED )
	{
		DCD_EP_Tx (pdev, HID_IN_EP, report, len);
	}
	return USBD_OK;
}

#ifdef HID_SUPPORT_IN_OUT 
void USBD_HID_RecvRegister(usbd_hid_recv_cb_t cb)
{
	usbd_hid_recv_cb = cb;
}
#endif

/**
  * @brief  USBD_HID_GetCfgDesc 
  *         return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_HID_GetCfgDesc (uint8_t speed, uint16_t *length)
{
	*length = sizeof (USBD_HID_CfgDesc);
	return USBD_HID_CfgDesc;
}

/**
  * @brief  USBD_HID_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_HID_DataIn (void  *pdev, 
                              uint8_t epnum)
{
	/* Ensure that the FIFO is empty before a new transfer, this condition could 
	be caused by  a new transfer before the end of the previous transfer */
	DCD_EP_Flush(pdev, HID_IN_EP);
	return USBD_OK;
}

#ifdef HID_SUPPORT_IN_OUT 
static uint8_t  USBD_HID_DataOut (void  *pdev, 
                              uint8_t epnum)
{
		usbd_hid_recv_cb(USBD_HID_RxBuf, HID_OUT_PACKET);
		
    DCD_EP_PrepareRx (pdev,
                     HID_OUT_EP,
                     USBD_HID_RxBuf,
                     HID_OUT_PACKET);
	
    return USBD_OK;
}
#endif

/**
  * @}
  */ 


/************************ (C) COPYRIGHT 2014 Megahuntmicro ****END OF FILE****/

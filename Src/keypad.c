//
// Created by codetector on 1/23/20.
//

#include "stm32f1xx_hal.h"
#include "usbd_hid.h"
#include "keypad.h"



static uint8_t  USBD_HID_Init(USBD_HandleTypeDef *pdev,
                              uint8_t cfgidx);

static uint8_t  USBD_HID_DeInit(USBD_HandleTypeDef *pdev,
                                uint8_t cfgidx);

static uint8_t  USBD_HID_Setup(USBD_HandleTypeDef *pdev,
                               USBD_SetupReqTypedef *req);

static uint8_t  *USBD_HID_GetFSCfgDesc(uint16_t *length);

static uint8_t  *USBD_HID_GetDeviceQualifierDesc(uint16_t *length);

static uint8_t  USBD_HID_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);

USBD_ClassTypeDef  KBD_USBD_HID =
        {
                USBD_HID_Init,
                USBD_HID_DeInit,
                USBD_HID_Setup,
                NULL, /*EP0_TxSent*/
                NULL, /*EP0_RxReady*/
                USBD_HID_DataIn, /*DataIn*/
                NULL, /*DataOut*/
                NULL, /*SOF */
                NULL,
                NULL,
                NULL,
                USBD_HID_GetFSCfgDesc,
                NULL,
                USBD_HID_GetDeviceQualifierDesc,
        };

/* USB HID device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_Desc[USB_HID_DESC_SIZ]  __ALIGN_END  =
        {
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
        };

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC]  __ALIGN_END =
        {
                USB_LEN_DEV_QUALIFIER_DESC,
                USB_DESC_TYPE_DEVICE_QUALIFIER,
                0x00,
                0x02,
                0x00,
                0x00,
                0x00,
                0x40,
                0x01,
                0x00,
        };

__ALIGN_BEGIN static uint8_t HID_Keyboard_ReportDesc[HID_KEYBOARD_REPORT_DESC_SIZE]  __ALIGN_END =
        {
                0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
                0x09, 0x06,        // Usage (Keyboard)
                0xA1, 0x01,        // Collection (Application)
                0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
                0x19, 0xE0,        //   Usage Minimum (0xE0)
                0x29, 0xE7,        //   Usage Maximum (0xE7)
                0x15, 0x00,        //   Logical Minimum (0)
                0x25, 0x01,        //   Logical Maximum (1)
                0x75, 0x01,        //   Report Size (1)
                0x95, 0x08,        //   Report Count (8)
                0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
                0x95, 0x01,        //   Report Count (1)
                0x75, 0x08,        //   Report Size (8)
                0x81, 0x03,        //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
                0x95, 0x06,        //   Report Count (6)
                0x75, 0x08,        //   Report Size (8)
                0x15, 0x00,        //   Logical Minimum (0)
                0x25, 0x65,        //   Logical Maximum (101)
                0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
                0x19, 0x00,        //   Usage Minimum (0x00)
                0x29, 0x65,        //   Usage Maximum (0x65)
                0x81, 0x00,        //   Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
                0xC0,              // End Collection
                // 45 bytes
        };

/* USB HID device FS Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_CfgFSDesc[USB_HID_CONFIG_DESC_SIZ]  __ALIGN_END =
        {
                0x09, /* bLength: Configuration Descriptor size */
                USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
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
                USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
                0x00,         /*bInterfaceNumber: Number of Interface*/
                0x00,         /*bAlternateSetting: Alternate setting*/
                0x01,         /*bNumEndpoints*/
                0x03,         /*bInterfaceClass: HID*/
                0x01,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
                0x01,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
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
                USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/

                HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
                0x03,          /*bmAttributes: Interrupt endpoint*/
                HID_EPIN_SIZE, /*wMaxPacketSize: 4 Byte max */
                0x00,
                HID_FS_BINTERVAL,          /*bInterval: Polling Interval */
                /* 34 */
        };


// Functions


/**
  * @brief  USBD_HID_Init
  *         Initialize the HID interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  /* Open EP IN */
  USBD_LL_OpenEP(pdev, HID_EPIN_ADDR, USBD_EP_TYPE_INTR, HID_EPIN_SIZE);
  pdev->ep_in[HID_EPIN_ADDR & 0xFU].is_used = 1U;

  pdev->pClassData = USBD_malloc(sizeof(USBD_HID_HandleTypeDef));

  if (pdev->pClassData == NULL)
  {
    return USBD_FAIL;
  }

  ((USBD_HID_HandleTypeDef *)pdev->pClassData)->state = HID_IDLE;

  return USBD_OK;
}

/**
  * @brief  USBD_HID_Init
  *         DeInitialize the HID layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_HID_DeInit(USBD_HandleTypeDef *pdev,
                                uint8_t cfgidx)
{
  /* Close HID EPs */
  USBD_LL_CloseEP(pdev, HID_EPIN_ADDR);
  pdev->ep_in[HID_EPIN_ADDR & 0xFU].is_used = 0U;

  /* FRee allocated memory */
  if (pdev->pClassData != NULL)
  {
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }

  return USBD_OK;
}

/**
  * @brief  USBD_HID_Setup
  *         Handle the HID specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_HID_Setup(USBD_HandleTypeDef *pdev,
                               USBD_SetupReqTypedef *req)
{
  USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *) pdev->pClassData;
  uint16_t len = 0U;
  uint8_t *pbuf = NULL;
  uint16_t status_info = 0U;
  USBD_StatusTypeDef ret = USBD_OK;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS :
      switch (req->bRequest)
      {
        case HID_REQ_SET_PROTOCOL:
          hhid->Protocol = (uint8_t)(req->wValue);
          break;

        case HID_REQ_GET_PROTOCOL:
          USBD_CtlSendData(pdev, (uint8_t *)(void *)&hhid->Protocol, 1U);
          break;

        case HID_REQ_SET_IDLE:
          hhid->IdleState = (uint8_t)(req->wValue >> 8);
          break;

        case HID_REQ_GET_IDLE:
          USBD_CtlSendData(pdev, (uint8_t *)(void *)&hhid->IdleState, 1U);
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
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            USBD_CtlSendData(pdev, (uint8_t *)(void *)&status_info, 2U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_GET_DESCRIPTOR:
          if (req->wValue >> 8 == HID_REPORT_DESC)
          {
            len = MIN(HID_KEYBOARD_REPORT_DESC_SIZE, req->wLength);
            pbuf = HID_Keyboard_ReportDesc;
          }
          else if (req->wValue >> 8 == HID_DESCRIPTOR_TYPE)
          {
            pbuf = USBD_HID_Desc;
            len = MIN(USB_HID_DESC_SIZ, req->wLength);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
            break;
          }
          USBD_CtlSendData(pdev, pbuf, len);
          break;

        case USB_REQ_GET_INTERFACE :
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            USBD_CtlSendData(pdev, (uint8_t *)(void *)&hhid->AltSetting, 1U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_SET_INTERFACE :
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            hhid->AltSetting = (uint8_t)(req->wValue);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
  }

  return ret;
}

static uint8_t  USBD_HID_DataIn(USBD_HandleTypeDef *pdev,
                                uint8_t epnum)
{

  /* Ensure that the FIFO is empty before a new transfer, this condition could
  be caused by  a new transfer before the end of the previous transfer */
  ((USBD_HID_HandleTypeDef *)pdev->pClassData)->state = HID_IDLE;
  return USBD_OK;
}


/**
* @brief  DeviceQualifierDescriptor
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
static uint8_t  *USBD_HID_GetDeviceQualifierDesc(uint16_t *length)
{
  *length = sizeof(USBD_HID_DeviceQualifierDesc);
  return USBD_HID_DeviceQualifierDesc;
}

/**
  * @brief  USBD_HID_GetCfgFSDesc
  *         return FS configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_HID_GetFSCfgDesc(uint16_t *length)
{
  *length = sizeof(USBD_HID_CfgFSDesc);
  return USBD_HID_CfgFSDesc;
}

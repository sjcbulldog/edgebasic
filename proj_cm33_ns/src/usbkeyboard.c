
/* emUSB-Host header file includes */
#include <USBH.h>
#include <USBH_HID.h>
#include <stdbool.h>
#include <stdint.h>
#include <cy_utils.h>
#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>  

#define USB_MAIN_TASK_MEMORY_REQ  (1024U)
#define USB_ISR_TASK_MEMORY_REQ   (1024U)

static USBH_NOTIFICATION_HOOK hook ;

static void usb_fatal()
{
    printf("APP_LOG: USBH fatal error \r\n");
    while(1);   
}

static void usbh_task(void* arg)
{
    CY_UNUSED_PARAMETER(arg);

    while(true)
    {
        USBH_Task();
    }
}

static void usbh_isr_task(void* arg)
{
    CY_UNUSED_PARAMETER(arg);

    while(true)
    {
        USBH_ISRTask();
    }
}

static void keyboardStateChange(USBH_HID_KEYBOARD_DATA *pKeyData)
{
    printf("APP_LOG: keyboardStateChange: Code: %d, Value %d, InterfaceID: %d \r\n", pKeyData->Code, pKeyData->Value, pKeyData->InterfaceID);
}

static void addDevice(uint8_t index)
{
    USBH_HID_DEVICE_INFO info ;

    USBH_HID_HANDLE handle = USBH_HID_Open(index) ;
    if (handle == USBH_HID_INVALID_HANDLE)
    {
        printf("APP_LOG: USBH_HID_Open failed for device %d \r\n", index);
        return ;
    }

    if (USBH_HID_GetDeviceInfo(handle, &info) != USBH_STATUS_SUCCESS)
    {
        printf("APP_LOG: USBH_HID_GetDeviceInfo failed for device %d \r\n", index);
        USBH_HID_Close(handle) ;
        return ;
    }

    printf("APP_LOG: USBH_HID_Open succeeded for device %d \r\n", index);
    printf("APP_LOG:   InputReportSize: %d \r\n", info.InputReportSize);
    printf("APP_LOG:   OutputReportSize: %d \r\n", info.OutputReportSize);
    printf("APP_LOG:   ProductID: %x \r\n", info.ProductId);
    printf("APP_LOG:   VendorID: %x \r\n", info.VendorId);
    printf("APP_LOG:   DevIndex: %d \r\n", info.DevIndex);
    printf("APP_LOG:   InterfaceID: %d \r\n", info.InterfaceID);
    printf("APP_LOG:   DeviceType: %d \r\n", info.DeviceType);
    printf("APP_LOG:   InterfaceNo: %d \r\n", info.InterfaceNo);
    printf("APP_LOG:   NumReportInfos: %d \r\n", info.NumReportInfos);
    for(int i = 0 ; i < info.NumReportInfos; i++)
    {
        printf("APP_LOG:       index: %d, ReportId %d: InputReportSize: %d, OutputReportSize: %d\r\n", 
                        i, info.ReportInfo[i].ReportId, info.ReportInfo[i].InputReportSize, info.ReportInfo[i].OutputReportSize);
    }

    printf("\r\n\r\n") ;

    USBH_HID_SetOnExKeyboardStateChange(keyboardStateChange) ;
}


static void usb_device_notify(void* usb_context, uint8_t usb_index, USBH_DEVICE_EVENT usb_event)
{
    CY_UNUSED_PARAMETER(usb_context);

    switch (usb_event)
    {
        case USBH_DEVICE_EVENT_ADD:
        {
            printf("USBH_DEVICE_EVENT_ADD: Device %d added \n", usb_index);
            addDevice(usb_index);
            break;
        }

        case USBH_DEVICE_EVENT_REMOVE:
        {
            printf("USBH_DEVICE_EVENT_REMOVE: Device %d removed \n", usb_index);
            break;
        }

        default:
        {
            printf("USBH_DEVICE_EVENT_INVALID: Device %d invalid event \n", usb_index);
            break;
        }
    }
}


void usbh_keyboard_task(void* arg)
{
    CY_UNUSED_PARAMETER(arg);

    BaseType_t rtos_task_status;

    printf("APP_LOG: Start USB configuration \r\n");
    
    /* Initialize USBH stack */
    USBH_Init();

    /* Create two tasks which are mandatory for USBH operation */
    rtos_task_status = xTaskCreate(usbh_task, "usbh_task",
                                   USB_MAIN_TASK_MEMORY_REQ, NULL, 
                                   configMAX_PRIORITIES - 1, NULL);

    if (pdPASS != rtos_task_status)
    {
        usb_fatal();
    }

    rtos_task_status = xTaskCreate(usbh_isr_task, "usbh_isr_task",
                                   USB_ISR_TASK_MEMORY_REQ, NULL, 
                                   configMAX_PRIORITIES - 2, NULL);

    if (pdPASS != rtos_task_status)
    {
        usb_fatal();
    }


    USBH_HID_Init() ;
    USBH_HID_AddNotification(&hook, usb_device_notify, NULL) ;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /* Initialize HID classes */
}

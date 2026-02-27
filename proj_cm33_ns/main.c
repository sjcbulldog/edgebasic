/*********************************************************************************
 * File Name        :   main.c
 *
 * Description      :   This is the source code for USB Host CDC Code Example
 *                      for ModusToolbox. 
 *
 * Related Document :   See README.md
 *
********************************************************************************
* Copyright 2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
 **********************************************************************************/

/* ModusToolbox header file includes*/
#include "retarget_io_init.h"
#include "cy_time.h"

/* FreeRTOS header file */
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "cyabs_rtos.h"
#include "cyabs_rtos_impl.h"

/* emUSB-Host header file includes */
#include "USBH.h"
#include "USBH_CDC.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define USB_CDC_TASK_MEMORY_REQ     (1500U)       /* In bytes */
#define USB_MAIN_TASK_MEMORY_REQ    (500U)        /* In bytes */  
#define USB_ISR_TASK_MEMORY_REQ     (500U)        /* In bytes */
#define PRINT_TASK_MEMORY_REQ       (500U)        /* In bytes */
#define WAIT_COUNT_VALUE            (10U)
#define DELAY_TASK_MS               (100U)
#define DELAY_ECHO_COMM_MS          (5000U)
#define USB_PACKET_SIZE             (64U)         /* In bytes */
#define USB_READ_TIMEOUT_MS         (50U)
#define USB_WRITE_TIMEOUT_MS        (50U)
#define USB_CDC_MSG                 ("Hello Infineon!\n")
#define USB_DEV_READY_TRUE          (1U)
#define USB_DEV_READY_FALSE         (0U)
#define USB_DEV_INDEX_DEFAULT       ((int8_t)(-1))
#define USB_PACKET_COUNT_RESET      (0U)

/* Enabling or disabling a MCWDT requires a wait time of upto 2 CLK_LF cycles  
 * to come into effect. This wait time value will depend on the actual CLK_LF  
 * frequency set by the BSP.
 */
#define LPTIMER_0_WAIT_TIME_USEC            (62U)

/* Define the LPTimer interrupt priority number. '1' implies highest priority. */
#define APP_LPTIMER_INTERRUPT_PRIORITY      (1U)

/* The timeout value in microsecond used to wait for core to be booted */
#define CM55_BOOT_WAIT_TIME_USEC    (10U)

/* App boot address for CM55 project */
#define CM55_APP_BOOT_ADDR          (CYMEM_CM33_0_m55_nvm_START + \
                                        CYBSP_MCUBOOT_HEADER_SIZE)


/***********************************************************************************
 *   Global Variables
 **********************************************************************************/
static USBH_NOTIFICATION_HOOK       usbh_cdc_notification;
static volatile uint8_t             device_ready;
static volatile int8_t              device_index;
static uint8_t                      data_buffer[USB_PACKET_SIZE];
static uint8_t                      packet_counter;

/* LPTimer HAL object */
static mtb_hal_lptimer_t lptimer_obj;

/* RTC HAL object */
static mtb_hal_rtc_t rtc_obj;


/*******************************************************************************
* Function Name: setup_clib_support
********************************************************************************
* Summary:
*    1. This function configures and initializes the Real-Time Clock (RTC).
*    2. It then initializes the RTC HAL object to enable CLIB support library 
*       to work with the provided Real-Time Clock (RTC) module.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void setup_clib_support(void)
{
    /* RTC Initialization */
    Cy_RTC_Init(&CYBSP_RTC_config);
    Cy_RTC_SetDateAndTime(&CYBSP_RTC_config);

    /* Initialize the ModusToolbox CLIB support library */
    mtb_clib_support_init(&rtc_obj);
}


/*******************************************************************************
* Function Name: lptimer_interrupt_handler
********************************************************************************
* Summary:
* Interrupt handler function for LPTimer instance. 
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void lptimer_interrupt_handler(void)
{
    mtb_hal_lptimer_process_interrupt(&lptimer_obj);
}


/*******************************************************************************
* Function Name: setup_tickless_idle_timer
********************************************************************************
* Summary:
* 1. This function first configures and initializes an interrupt for LPTimer.
* 2. Then it initializes the LPTimer HAL object to be used in the RTOS 
*    tickless idle mode implementation to allow the device enter deep sleep 
*    when idle task runs. LPTIMER_0 instance is configured for CM33 CPU.
* 3. It then passes the LPTimer object to abstraction RTOS library that 
*    implements tickless idle mode
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void setup_tickless_idle_timer()
{
    /* Interrupt configuration structure for LPTimer */
    cy_stc_sysint_t lptimer_intr_cfg =
    {
        .intrSrc = CYBSP_CM33_LPTIMER_0_IRQ,
        .intrPriority = APP_LPTIMER_INTERRUPT_PRIORITY
    };

    /* Initialize the LPTimer interrupt and specify the interrupt handler. */
    cy_en_sysint_status_t interrupt_init_status = 
                                    Cy_SysInt_Init(&lptimer_intr_cfg, 
                                                    lptimer_interrupt_handler);
    
    /* LPTimer interrupt initialization failed. Stop program execution. */
    if(CY_SYSINT_SUCCESS != interrupt_init_status)
    {
        handle_app_error();
    }

    /* Enable NVIC interrupt. */
    NVIC_EnableIRQ(lptimer_intr_cfg.intrSrc);

    /* Initialize the MCWDT block */
    cy_en_mcwdt_status_t mcwdt_init_status = 
                                    Cy_MCWDT_Init(CYBSP_CM33_LPTIMER_0_HW, 
                                                &CYBSP_CM33_LPTIMER_0_config);

    /* MCWDT initialization failed. Stop program execution. */
    if(CY_MCWDT_SUCCESS != mcwdt_init_status)
    {
        handle_app_error();
    }
  
    /* Enable MCWDT instance */
    Cy_MCWDT_Enable(CYBSP_CM33_LPTIMER_0_HW,
                    CY_MCWDT_CTR_Msk, 
                    LPTIMER_0_WAIT_TIME_USEC);

    /* Setup LPTimer using the HAL object and desired configuration as defined
     * in the device configurator. */
    cy_rslt_t result = mtb_hal_lptimer_setup(&lptimer_obj, 
                                            &CYBSP_CM33_LPTIMER_0_hal_config);
    
    /* LPTimer setup failed. Stop program execution. */
    if(CY_RSLT_SUCCESS != result)
    {
        handle_app_error();
    }

    /* Pass the LPTimer object to abstraction RTOS library that implements 
     * tickless idle mode 
     */
    cyabs_rtos_set_lptimer(&lptimer_obj);
}


/***********************************************************************************
*  Function Name: usbh_task
***********************************************************************************
* Summary:
* The function is responsible for calling USBH_Task(). During the execution of the
* program, exit from the infinite loop is not expected. 
* 
* The USBH_Task() manages the internal software timers of the emUSB-Host stack.
*
* Parameters:
* arg - unused
* 
* Return:
* void
*
**********************************************************************************/
static void usbh_task(void* arg)
{
    CY_UNUSED_PARAMETER(arg);

    while(true)
    {
        USBH_Task();
    }
}

/***********************************************************************************
*   Function Name: usbh_isr_task
***********************************************************************************
* Summary:
* The function is responsible for calling USBH_ISRTask(). During the execution of 
* the program, exit from the infinite loop is not expected. 
*
* The USBH_ISRTask() process the events trigerred from the interrupt handler.
*
* Parameters:
* arg - is not used in this function, is required by FreeRTOS
* 
* Return:
* void
*
**********************************************************************************/
static void usbh_isr_task(void* arg)
{
    CY_UNUSED_PARAMETER(arg);

    while(true)
    {
        USBH_ISRTask();
    }
}


/***********************************************************************************
* Function Name: usb_device_notify
***********************************************************************************
* Summary:
* This function performs registration/de-registration of USB devices on USB host.
* 
* Parameters:
* usb_context  :   Pointer to a context passed by the user in the call to one of
*                  the register functions.
* usb_index    :   Zero based index of the device that was added or removed.
*                  First device has index 0, second one has index 1, etc
* usb_event    :   Enum USBH_DEVICE_EVENT which gives information about the 
*                  event that occurred.
* 
* Return:
* void
*
**********************************************************************************/
static void usb_device_notify(void* usb_context, uint8_t usb_index, USBH_DEVICE_EVENT usb_event)
{
    CY_UNUSED_PARAMETER(usb_context);

    switch (usb_event)
    {
        case USBH_DEVICE_EVENT_ADD:
        {
            printf("\n\n");
            USBH_Logf_Application("======================== APP_LOG: Device added [%d]" 
                                  "========================\n\n\n\n", usb_index);
            device_index    = usb_index;
            device_ready    = USB_DEV_READY_TRUE;
            packet_counter  = USB_PACKET_COUNT_RESET;
            break;
        }

        case USBH_DEVICE_EVENT_REMOVE:
        {
            printf("\n\n");
            USBH_Logf_Application("======================== APP_LOG: Device removed [%d]"
                                  "======================\n\n\n\n", usb_index);
            device_ready    = USB_DEV_READY_FALSE;
            device_index    = USB_DEV_INDEX_DEFAULT;
            break;
        }

        default:
        {
            printf("\n\n");
            USBH_Logf_Application("======================== APP_LOG: Invalid event [%d]"
                                  "=======================\n\n\n\n", usb_index);
            break;
        }
    }
}


/***********************************************************************************
* Function Name: device_task
***********************************************************************************
* Summary:
* This function is responsible for retrieval of the USB device information and 
* configures the CDC device to start the echo communication. It contains the 
* print logs for the data packets as well as the echo data stream. 
* 
* Parameters:
* void
* 
* Return:
* void
*
**********************************************************************************/
static void device_task(void)
{
    /* Open the device, the device index is retrieved from the notification callback. */
    USBH_CDC_HANDLE             device_handle = USBH_CDC_Open(device_index);

    if (device_handle)
    {
        USBH_CDC_DEVICE_INFO    usb_device_info;
        USBH_STATUS             usb_status;
        unsigned long           numBytes;

        /* Configure the CDC device. */
        USBH_CDC_SetTimeouts(device_handle, USB_READ_TIMEOUT_MS, USB_WRITE_TIMEOUT_MS);
        USBH_CDC_AllowShortRead(device_handle, 1);
        USBH_CDC_SetCommParas(device_handle, USBH_CDC_BAUD_115200, USBH_CDC_BITS_8,
                                USBH_CDC_STOP_BITS_1, USBH_CDC_PARITY_NONE);

        /* Retrieve the information about the CDC device */ 
        USBH_CDC_GetDeviceInfo(device_handle, &usb_device_info);  

        printf("Initiating echo communication: Packet Number: %u\n", packet_counter);  

        printf("Vendor  ID = 0x%.4X\n", usb_device_info.VendorId);
        printf("Product ID = 0x%.4X\n", usb_device_info.ProductId);

        printf("Writing to the device %s", USB_CDC_MSG);
        USBH_CDC_Write(device_handle, (const uint8_t *)USB_CDC_MSG, (uint32_t)(sizeof(USB_CDC_MSG) - 1), &numBytes);
        printf("Reading from the device\n");
        usb_status = USBH_CDC_Read(device_handle, data_buffer, sizeof(data_buffer), &numBytes);

        if (usb_status != USBH_STATUS_SUCCESS)
        {
            printf("Error occurred during reading from device\n");
        }
        else
        {
            data_buffer[numBytes] = 0;
            printf("Received: %s \n",(char *)data_buffer);
            printf("Communication of USB Host with USB Device Successful\n");
        }
        
        printf("Re-initiating echo communication in 5 seconds\n\n\n\n");
        Cy_SysLib_Delay(DELAY_ECHO_COMM_MS);
        packet_counter++;
    }
}


/***********************************************************************************
*  Function Name: usbh_cdc_task
***********************************************************************************
* Summary:
* Initializes emUSB-Host stack and registers all necessary tasks.
*
* Parameters:
* arg - unused
* 
* Return:
* void
*
**********************************************************************************/
static void usbh_cdc_task(void* arg)
{
    CY_UNUSED_PARAMETER(arg);

    USBH_STATUS usb_status;
    BaseType_t rtos_task_status;

    uint32_t wait_counter = WAIT_COUNT_VALUE;

    printf("APP_LOG: Start USB configuration \r\n");
    
    /* Initialize USBH stack */
    USBH_Init();

    /* Create two tasks which are mandatory for USBH operation */
    printf("APP_LOG: Register usbh_task task \r\n");
    rtos_task_status = xTaskCreate(usbh_task, "usbh_task",
                                   USB_MAIN_TASK_MEMORY_REQ, NULL, 
                                   configMAX_PRIORITIES - 1, NULL);

    if (pdPASS != rtos_task_status)
    {
        handle_app_error();
    }

    printf("APP_LOG: Register usbh_isr_task task \r\n\n");
    rtos_task_status = xTaskCreate(usbh_isr_task, "usbh_isr_task",
                                   USB_ISR_TASK_MEMORY_REQ, NULL, 
                                   configMAX_PRIORITIES - 2, NULL);

    if (pdPASS != rtos_task_status)
    {
        handle_app_error();
    }

    printf("APP_LOG: Initialize CDC classes \r\n\n");

    /* Initialize CDC classes */
    USBH_CDC_Init();

    USBH_CDC_SetConfigFlags(USBH_CDC_IGNORE_INT_EP | USBH_CDC_DISABLE_INTERFACE_CHECK);
    usb_status = USBH_CDC_AddNotification(&usbh_cdc_notification, usb_device_notify, NULL);

    if (USBH_STATUS_SUCCESS != usb_status)
    {
        handle_app_error();
    }

    printf("APP_LOG: Waiting for a USB CDC device \r\n\n");

    for (;;)
    {
        if (!wait_counter)
        {
            wait_counter = WAIT_COUNT_VALUE;
            /* Wait State */
            Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
        }
        else
        {
            wait_counter --;
        }

        vTaskDelay(pdMS_TO_TICKS(DELAY_TASK_MS));
        if (device_ready)
        {
            device_task();
        }
    }
}


/***********************************************************************************
 *  Function Name: main
 ***********************************************************************************
* Summary:
* This is the main function for CM33 CPU NSPE.
*    1. Initializes the device and board peripherals.
*    2. Initializes the RTOS handle for usbh_cdc_task.
*    3. Starts the RTOS task scheduler.  
*
* Parameters:
* void
*
* Return:
* int
*
**********************************************************************************/
int main(void)
{
    cy_rslt_t result;
    BaseType_t rtos_task_status;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (CY_RSLT_SUCCESS != result)
    {
        handle_app_error();
    }

    /* Setup CLIB support library. */
    setup_clib_support();

    /* Setup the LPTimer instance for CM33 CPU. */
    setup_tickless_idle_timer();

    init_retarget_io();

    /* Enable CM55. */
    /* CM55_APP_BOOT_ADDR must be updated if CM55 memory layout is changed.*/
    Cy_SysEnableCM55(MXCM55, CM55_APP_BOOT_ADDR, CM55_BOOT_WAIT_TIME_USEC);

    /* Enable global interrupts */
    __enable_irq();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf( "=================================================================\r\n\n"
             "PSOC Edge MCU: CDC echo application using emUSB-Host\r\n\n"
             "================================================================\r\n\n");

    rtos_task_status = xTaskCreate(usbh_cdc_task, "usbh_cdc_task", USB_CDC_TASK_MEMORY_REQ, NULL, 
                                   configMAX_PRIORITIES - 1, NULL);
    if (pdPASS != rtos_task_status)
    {
        handle_app_error();
    }

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();

    /* This line will never be reached */
    printf("APP_LOG: Error: FreeRTOS doesn't start\r\n");
}


/* [] END OF FILE */
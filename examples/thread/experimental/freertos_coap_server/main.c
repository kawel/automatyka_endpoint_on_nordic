/**
 * Copyright (c) 2017 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 *
 * @defgroup freertos_coap_server_example_main main.c
 * @{
 * @ingroup freertos_coap_server_example
 *
 * @brief Thread CoAP server example with FreeRTOS Application main file.
 *
 * This file contains the source code for a sample application using Thread CoAP server and FreeRTOS.
 *
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "app_error.h"
#include "app_timer.h"
#include "bsp_thread.h"
#include "FreeRTOS.h"
#include "nordic_common.h"
#include "nrf_drv_clock.h"
#include "sdk_errors.h"
#include "task.h"
#include "timers.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include <openthread/openthread.h>
#include <openthread/diag.h>
#include <openthread/coap.h>
#include <openthread/cli.h>
#include <openthread/platform/alarm.h>
#include <openthread/platform/platform.h>
#include <openthread/platform/platform-softdevice.h>

#define THREAD_STACK_TASK_STACK_SIZE     (( 1024 * 8 ) / sizeof(StackType_t))   /**< FreeRTOS task stack size is determined in multiples of StackType_t. */
#define LOG_TASK_STACK_SIZE              ( 1024 / sizeof(StackType_t))          /**< FreeRTOS task stack size is determined in multiples of StackType_t. */
#define THREAD_STACK_TASK_PRIORITY       2
#define LOG_TASK_PRIORITY                1
#define LED1_TASK_PRIORITY               1
#define LED2_TASK_PRIORITY               1
#define PROVISIONING_LED_BLINK_INTERVAL  100
#define LED1_BLINK_INTERVAL              427
#define LED2_BLINK_INTERVAL              472
#define PROVISIONING_EXPIRY_TIME         5000

APP_TIMER_DEF(m_provisioning_timer);
APP_TIMER_DEF(m_led_timer);

static void light_request_handler(void                * p_context,
                                  otCoapHeader        * p_header,
                                  otMessage           * p_message,
                                  const otMessageInfo * p_message_info);

static void provisioning_request_handler(void                * p_context,
                                         otCoapHeader        * p_header,
                                         otMessage           * p_message,
                                         const otMessageInfo * p_message_info);

typedef enum
{
    DEVICE_TYPE_REMOTE_CONTROL,
    DEVICE_TYPE_LIGHT
} device_type_t;

typedef enum
{
    LIGHT_OFF = 0,
    LIGHT_ON,
    LIGHT_TOGGLE
} light_command_t;

typedef struct
{
    otInstance     * p_ot_instance;         /**< A pointer to the OpenThread instance. */
    TaskHandle_t     thread_stack_task;     /**< Thread stack task handle */
    TaskHandle_t     logger_task;           /**< Definition of Logger task. */
    TaskHandle_t     led1_task;             /**< LED1 task handle*/
    TaskHandle_t     led2_task;             /**< LED2 task handle*/
    bool             enable_provisioning;   /**< Information if provisioning is enabled. */
    uint32_t         provisioning_expiry;   /**< Provisioning timeout time. */
    otCoapResource   provisioning_resource; /**< CoAP provisioning resource. */
    otCoapResource   light_resource;        /**< CoAP light resource. */
    bool             led_blinking_is_on;    /**< Indicates if animation is enabled. */
} application_t;

application_t m_app =
{
    .p_ot_instance         = NULL,
    .thread_stack_task     = NULL,
    .logger_task           = NULL,
    .led1_task             = NULL,
    .led2_task             = NULL,
    .enable_provisioning   = false,
    .provisioning_expiry   = 0,
    .provisioning_resource = {"provisioning", provisioning_request_handler, NULL, NULL},
    .light_resource        = {"light", light_request_handler, NULL, NULL},
};


/***************************************************************************************************
 * @section CoAP
 **************************************************************************************************/

static void light_on(void)
{
    vTaskResume(m_app.led1_task);
    vTaskResume(m_app.led2_task);
}

static void light_off(void)
{
    vTaskSuspend(m_app.led1_task);
    LEDS_OFF(BSP_LED_2_MASK);
    vTaskSuspend(m_app.led2_task);
    LEDS_OFF(BSP_LED_3_MASK);
}

static void light_toggle(void)
{
    if ((eTaskGetState(m_app.led1_task) == eSuspended) && (eTaskGetState(m_app.led2_task) == eSuspended))
    {
        light_on();
    }
    else
    {
        light_off();
    }
}

static void provisioning_disable(otInstance * p_instance)
{
    m_app.enable_provisioning = false;
    m_app.provisioning_expiry = 0;
    app_timer_stop(m_provisioning_timer);
    app_timer_stop(m_led_timer);

    if (m_app.led_blinking_is_on)
    {
        light_on();
    }
    else
    {
        light_off();
    }
}

static void provisioning_enable(otInstance * p_instance)
{
    uint32_t err_code;

    m_app.enable_provisioning = true;
    m_app.provisioning_expiry = otPlatAlarmGetNow() + PROVISIONING_EXPIRY_TIME;
    m_app.led_blinking_is_on  = !(eTaskGetState(m_app.led1_task) == eSuspended);

    light_off();

    err_code = app_timer_start(m_provisioning_timer,
                               APP_TIMER_TICKS(PROVISIONING_EXPIRY_TIME),
                               p_instance);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_led_timer, APP_TIMER_TICKS(PROVISIONING_LED_BLINK_INTERVAL), NULL);
    APP_ERROR_CHECK(err_code);
}

static void light_response_send(void                * p_context,
                                otCoapHeader        * p_request_header,
                                const otMessageInfo * p_message_info)
{
    otError      error = OT_ERROR_NONE;
    otCoapHeader header;
    otMessage  * p_response;

    do
    {
        otCoapHeaderInit(&header, OT_COAP_TYPE_ACKNOWLEDGMENT, OT_COAP_CODE_CHANGED);
        otCoapHeaderSetMessageId(&header, otCoapHeaderGetMessageId(p_request_header));
        otCoapHeaderSetToken(&header,
                             otCoapHeaderGetToken(p_request_header),
                             otCoapHeaderGetTokenLength(p_request_header));

        p_response = otCoapNewMessage(p_context, &header);
        if (p_response == NULL)
        {
            break;
        }

        error = otCoapSendResponse(p_context, p_response, p_message_info);

    } while (false);

    if (error != OT_ERROR_NONE && p_response != NULL)
    {
        otMessageFree(p_response);
    }
}

static void light_request_handler(void                * p_context,
                                  otCoapHeader        * p_header,
                                  otMessage           * p_message,
                                  const otMessageInfo * p_message_info)
{
    (void)p_message;
    uint8_t command;

    do
    {
        if (otCoapHeaderGetType(p_header) != OT_COAP_TYPE_CONFIRMABLE &&
            otCoapHeaderGetType(p_header) != OT_COAP_TYPE_NON_CONFIRMABLE)
        {
            break;
        }

        if (otCoapHeaderGetCode(p_header) != OT_COAP_CODE_PUT)
        {
            break;
        }

        if (otMessageRead(p_message, otMessageGetOffset(p_message), &command, 1) != 1)
        {
            NRF_LOG_INFO("light handler - missing command\r\n");
        }

        switch (command)
        {
            case LIGHT_ON:
                light_on();
                break;

            case LIGHT_OFF:
                light_off();
                break;

            case LIGHT_TOGGLE:
                light_toggle();
                break;

            default:
                break;
        }

        if (otCoapHeaderGetType(p_header) == OT_COAP_TYPE_CONFIRMABLE)
        {
            light_response_send(p_context, p_header, p_message_info);
        }

    } while (false);
}

static otError provisioning_response_send(void                * p_context,
                                          otCoapHeader        * p_request_header,
                                          uint8_t               device_type,
                                          const otMessageInfo * p_message_info)
{
    otError      error = OT_ERROR_NO_BUFS;
    otCoapHeader header;
    otMessage  * p_response;

    do
    {
        otCoapHeaderInit(&header, OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_CONTENT);
        otCoapHeaderSetToken(&header,
                             otCoapHeaderGetToken(p_request_header),
                             otCoapHeaderGetTokenLength(p_request_header));
        otCoapHeaderSetPayloadMarker(&header);

        p_response = otCoapNewMessage(p_context, &header);
        if (p_response == NULL)
        {
            break;
        }

        error = otMessageAppend(p_response, &device_type, 1);
        if (error != OT_ERROR_NONE)
        {
            break;
        }

        error = otMessageAppend(p_response, otThreadGetMeshLocalEid(p_context), sizeof(otIp6Address));
        if (error != OT_ERROR_NONE)
        {
            break;
        }

        error = otCoapSendResponse(p_context, p_response, p_message_info);

    } while (false);

    if (error != OT_ERROR_NONE && p_response != NULL)
    {
        otMessageFree(p_response);
    }

    return error;
}

static void provisioning_request_handler(void                * p_context,
                                         otCoapHeader        * p_header,
                                         otMessage           * p_message,
                                         const otMessageInfo * p_message_info)
{
    (void)p_message;
    otMessageInfo message_info;

    if (!m_app.enable_provisioning)
    {
        return;
    }

    if (otCoapHeaderGetType(p_header) == OT_COAP_TYPE_NON_CONFIRMABLE &&
        otCoapHeaderGetCode(p_header) == OT_COAP_CODE_GET)
    {
        message_info = *p_message_info;
        memset(&message_info.mSockAddr, 0, sizeof(message_info.mSockAddr));

        if (provisioning_response_send(p_context, p_header, DEVICE_TYPE_LIGHT, &message_info) ==
            OT_ERROR_NONE)
        {
            provisioning_disable(p_context);
        }
    }
}


/***************************************************************************************************
 * @section State change handling
 **************************************************************************************************/

static void role_change_handler(void * p_context, otDeviceRole role)
{
    switch(role)
    {
        case OT_DEVICE_ROLE_CHILD:
        case OT_DEVICE_ROLE_ROUTER:
        case OT_DEVICE_ROLE_LEADER:
            break;

        case OT_DEVICE_ROLE_DISABLED:
        case OT_DEVICE_ROLE_DETACHED:
        default:
            provisioning_disable(p_context);
            break;
    }
}

static void state_changed_callback(uint32_t flags, void * p_context)
{
    if (flags & OT_CHANGED_THREAD_ROLE)
    {
        role_change_handler(p_context, otThreadGetDeviceRole(p_context));
    }

    NRF_LOG_INFO("State changed! Flags: 0x%08x Current role: %d\r\n", flags, otThreadGetDeviceRole(p_context));
}


/***************************************************************************************************
 * @section Buttons
 **************************************************************************************************/

static void bsp_event_handler(bsp_event_t event)
{
    switch (event)
    {
        case BSP_EVENT_KEY_0:
            break;

        case BSP_EVENT_KEY_1:
            break;

        case BSP_EVENT_KEY_2:
            break;

        case BSP_EVENT_KEY_3:
            provisioning_enable(m_app.p_ot_instance);
            break;

        default:
            return;
    }
}


/***************************************************************************************************
 * @section Timers
 **************************************************************************************************/

static void provisioning_timer_handler(void * p_context)
{
    provisioning_disable(p_context);
}

static void led_timer_handler(void * p_context)
{
    (void)p_context;

    if (m_app.enable_provisioning)
    {
        LEDS_INVERT(BSP_LED_2_MASK);
    }
    else
    {
        LEDS_OFF(BSP_LED_2_MASK);
    }
}


/***************************************************************************************************
 * @section Initialization
 **************************************************************************************************/

static void thread_init(void)
{
    otInstance * p_instance;

    PlatformInit(0, NULL);

    p_instance = otInstanceInit();
    assert(p_instance);

    otCliUartInit(p_instance);

    NRF_LOG_INFO("Thread version: %s\r\n", (uint32_t)otGetVersionString());
    NRF_LOG_INFO("Network name:   %s\r\n", (uint32_t)otThreadGetNetworkName(p_instance));

    assert(otSetStateChangedCallback(p_instance, &state_changed_callback, p_instance) == OT_ERROR_NONE);

    if (!otDatasetIsCommissioned(p_instance))
    {
        assert(otLinkSetChannel(p_instance, THREAD_CHANNEL) == OT_ERROR_NONE);
        assert(otLinkSetPanId(p_instance, THREAD_PANID) == OT_ERROR_NONE);
    }

    assert(otIp6SetEnabled(p_instance, true) == OT_ERROR_NONE);
    assert(otThreadSetEnabled(p_instance, true) == OT_ERROR_NONE);

    m_app.p_ot_instance = p_instance;
}

static void coap_init()
{
    m_app.light_resource.mContext        = m_app.p_ot_instance;
    m_app.provisioning_resource.mContext = m_app.p_ot_instance;

    assert(otCoapStart(m_app.p_ot_instance, OT_DEFAULT_COAP_PORT) == OT_ERROR_NONE);
    assert(otCoapAddResource(m_app.p_ot_instance, &m_app.light_resource) == OT_ERROR_NONE);
    assert(otCoapAddResource(m_app.p_ot_instance, &m_app.provisioning_resource) == OT_ERROR_NONE);
}

static void timer_init(void)
{
    uint32_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    app_timer_create(&m_provisioning_timer, APP_TIMER_MODE_SINGLE_SHOT, provisioning_timer_handler);
    app_timer_create(&m_led_timer, APP_TIMER_MODE_REPEATED, led_timer_handler);
}

static void thread_bsp_init(void)
{
    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_thread_init(m_app.p_ot_instance);
    APP_ERROR_CHECK(err_code);
}

static void leds_init(void)
{
    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);

    app_timer_start(m_led_timer, APP_TIMER_TICKS(PROVISIONING_LED_BLINK_INTERVAL), NULL);
}

static void thread_stack_task(void * arg)
{
    UNUSED_PARAMETER(arg);

    thread_init();
    coap_init();

    timer_init();
    thread_bsp_init();
    leds_init();

    while (1)
    {
        otTaskletsProcess(m_app.p_ot_instance);
        PlatformProcessDrivers(m_app.p_ot_instance);

        if (!otTaskletsArePending(m_app.p_ot_instance))
        {
            // Suspend OpenThread task.
            vTaskSuspend(NULL);
        }
    }
}


/***************************************************************************************************
 * @section Leds
 **************************************************************************************************/

static void led1_task(void * arg)
{
    UNUSED_PARAMETER(arg);

    while(1)
    {
        LEDS_INVERT(BSP_LED_2_MASK);
        vTaskDelay(LED1_BLINK_INTERVAL);
    }
}

static void led2_task(void * arg)
{
    UNUSED_PARAMETER(arg);

    while(1)
    {
        LEDS_INVERT(BSP_LED_3_MASK);
        vTaskDelay(LED2_BLINK_INTERVAL);
    }
}


/***************************************************************************************************
 * @section Idle hook
 **************************************************************************************************/

void vApplicationIdleHook( void )
{
    if (m_app.thread_stack_task)
    {
        vTaskResume(m_app.thread_stack_task);
    }

    vTaskResume(m_app.logger_task);
}


#if NRF_LOG_ENABLED
/**@brief Task for handling the logger.
 *
 * @details This task is responsible for processing log entries if logs are deferred.
 *          Task flushes all log entries and suspends. It is resumed by idle task hook.
 *
 * @param[in]   arg   Pointer used for passing some arbitrary information (context) from the
 *                    osThreadCreate() call to the task.
 */
static void logger_task(void * arg)
{
    UNUSED_PARAMETER(arg);

    while (1)
    {
        NRF_LOG_FLUSH();

        // Suspend logger task.
        vTaskSuspend(NULL);
    }
}
#endif //NRF_LOG_ENABLED


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the clock.
 */
static void clock_init(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
}


int main(void)
{
    clock_init();
    log_init();

    // Start thread stack execution.
    if (pdPASS != xTaskCreate(thread_stack_task, "THR", THREAD_STACK_TASK_STACK_SIZE, NULL, 2, &m_app.thread_stack_task))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

#if NRF_LOG_ENABLED
    // Start execution.
    if (pdPASS != xTaskCreate(logger_task, "LOG", LOG_TASK_STACK_SIZE, NULL, 1, &m_app.logger_task))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
#endif //NRF_LOG_ENABLED

    // Start execution.
    if (pdPASS != xTaskCreate(led1_task, "LED1", configMINIMAL_STACK_SIZE, NULL, 1, &m_app.led1_task))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    // Start execution.
    if (pdPASS != xTaskCreate(led2_task, "LED2", configMINIMAL_STACK_SIZE, NULL, 1, &m_app.led2_task))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }


    /* Start FreeRTOS scheduler. */
    vTaskStartScheduler();

    while (true)
    {
        /* FreeRTOS should not be here... FreeRTOS goes back to the start of stack
         * in vTaskStartScheduler function. */
    }
}

/**
 *@}
 **/

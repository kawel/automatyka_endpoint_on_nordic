/* Copyright (c) 2016-2017 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup mtd_coap_client_example_main main.c
 * @{
 * @ingroup mtd_coap_client_example_example
 * @brief MTD CoAP Client Example Application main file.
 *
 * @details This example demonstrates a low power CoAP client application that enables to control
 *          BSP_LED_0 on a board with related Simple CoAP Server application via CoAP messages.
 *
 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "bsp_thread.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include <openthread/openthread.h>
#include <openthread/diag.h>
#include <openthread/coap.h>
#include <openthread/platform/platform.h>

#define DEFAULT_CHILD_TIMEOUT    (4 * THREAD_POLL_PERIOD)
#define DEFAULT_POLL_PERIOND     (1000 * THREAD_POLL_PERIOD)
#define COAP_POLL_PERIOD         500

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
    otInstance   * p_ot_instance;       /**< A pointer to the OpenThread instance. */
    otIp6Address   peer_address;        /**< An address of a related server node. */
    bool           multicast_light_on;  /**< Information which multicast command should be sent next. */
} application_t;

static application_t m_app =
{
    .p_ot_instance      = NULL,
    .peer_address       = { .mFields.m8 = { 0 } },
    .multicast_light_on = false,
};

static const otIp6Address m_unspecified_ipv6 = { .mFields.m8 = { 0 } };

/***************************************************************************************************
 * @section CoAP
 **************************************************************************************************/

static void light_response_handler(void                * p_context,
                                   otCoapHeader        * p_header,
                                   otMessage           * p_message,
                                   const otMessageInfo * p_message_info,
                                   otError               result)
{
    (void)p_context;
    (void)p_header;
    (void)p_message;

    otLinkSetPollPeriod(m_app.p_ot_instance, DEFAULT_POLL_PERIOND);

    if (result == OT_ERROR_NONE)
    {
        NRF_LOG_INFO("Received light control response.\r\n");
    }
    else
    {
        NRF_LOG_INFO("Failed to receive response: %d\r\n", result);
        m_app.peer_address = m_unspecified_ipv6;
    }
}

static void unicast_light_request_send(otInstance * p_instance, uint8_t command)
{
    otError       error = OT_ERROR_NONE;
    otMessage   * p_message;
    otMessageInfo messageInfo;
    otCoapHeader  header;

    do
    {
        otCoapHeaderInit(&header, OT_COAP_TYPE_CONFIRMABLE, OT_COAP_CODE_PUT);
        otCoapHeaderGenerateToken(&header, 2);
        otCoapHeaderAppendUriPathOptions(&header, "light");
        otCoapHeaderSetPayloadMarker(&header);

        p_message = otCoapNewMessage(p_instance, &header);
        if (p_message == NULL)
        {
            NRF_LOG_INFO("Failed to allocate message for CoAP Request\r\n");
            break;
        }

        error = otMessageAppend(p_message, &command, sizeof(command));
        if (error != OT_ERROR_NONE)
        {
            break;
        }

        memset(&messageInfo, 0, sizeof(messageInfo));
        messageInfo.mInterfaceId = OT_NETIF_INTERFACE_ID_THREAD;
        messageInfo.mPeerPort = OT_DEFAULT_COAP_PORT;
        memcpy(&messageInfo.mPeerAddr, &m_app.peer_address, sizeof(messageInfo.mPeerAddr));

        error = otCoapSendRequest(p_instance,
                                  p_message,
                                  &messageInfo,
                                  &light_response_handler,
                                  p_instance);
    } while (false);

    if (error != OT_ERROR_NONE && p_message != NULL)
    {
        NRF_LOG_INFO("Failed to send CoAP Request: %d\r\n", error);
        otMessageFree(p_message);
    }
}

static void multicast_light_request_send(otInstance * p_instance, uint8_t command)
{
    otError       error = OT_ERROR_NONE;
    otMessage   * p_message;
    otMessageInfo messageInfo;
    otCoapHeader  header;

    do
    {
        otCoapHeaderInit(&header, OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_PUT);
        otCoapHeaderAppendUriPathOptions(&header, "light");
        otCoapHeaderSetPayloadMarker(&header);

        p_message = otCoapNewMessage(p_instance, &header);
        if (p_message == NULL)
        {
            NRF_LOG_INFO("Failed to allocate message for CoAP Request\r\n");
            break;
        }

        error = otMessageAppend(p_message, &command, sizeof(command));
        if (error != OT_ERROR_NONE)
        {
            break;
        }

        memset(&messageInfo, 0, sizeof(messageInfo));
        messageInfo.mInterfaceId = OT_NETIF_INTERFACE_ID_THREAD;
        messageInfo.mPeerPort = OT_DEFAULT_COAP_PORT;
        otIp6AddressFromString("FF03::1", &messageInfo.mPeerAddr);

        error = otCoapSendRequest(p_instance, p_message, &messageInfo, NULL, NULL);
    } while (false);

    if (error != OT_ERROR_NONE && p_message != NULL)
    {
        NRF_LOG_INFO("Failed to send CoAP Request: %d\r\n", error);
        otMessageFree(p_message);
    }
}

static void provisioning_response_handler(void                * p_context,
                                          otCoapHeader        * p_header,
                                          otMessage           * p_message,
                                          const otMessageInfo * p_message_info,
                                          otError               result)
{
    (void)p_context;
    (void)p_header;

    uint8_t peer_type;

    otLinkSetPollPeriod(m_app.p_ot_instance, DEFAULT_POLL_PERIOND);

    if (result == OT_ERROR_NONE)
    {
        if ((otMessageRead(p_message, otMessageGetOffset(p_message), &peer_type, 1) == 1) &&
            (peer_type == DEVICE_TYPE_LIGHT))
        {
            otMessageRead(p_message,
                          otMessageGetOffset(p_message) + 1,
                          &m_app.peer_address,
                          sizeof(m_app.peer_address));
        }
    }
    else
    {
        NRF_LOG_INFO("Provisioning failed: %d\r\n", result);
    }
}

static void provisioning_request_send(otInstance * p_instance)
{
    otError       error = OT_ERROR_NONE;
    otCoapHeader  header;
    otMessage   * p_request;
    otMessageInfo aMessageInfo;

    do
    {
        otCoapHeaderInit(&header, OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_GET);
        otCoapHeaderGenerateToken(&header, 2);
        otCoapHeaderAppendUriPathOptions(&header, "provisioning");

        p_request = otCoapNewMessage(p_instance, &header);
        if (p_request == NULL)
        {
            break;
        }

        memset(&aMessageInfo, 0, sizeof(aMessageInfo));
        aMessageInfo.mInterfaceId = OT_NETIF_INTERFACE_ID_THREAD;
        aMessageInfo.mPeerPort = OT_DEFAULT_COAP_PORT;
        otIp6AddressFromString("FF03::1", &aMessageInfo.mPeerAddr);

        error = otCoapSendRequest(p_instance,
                                  p_request,
                                  &aMessageInfo,
                                  provisioning_response_handler,
                                  p_instance);
    } while (false);

    if (error != OT_ERROR_NONE && p_request != NULL)
    {
        otMessageFree(p_request);
    }
}

/***************************************************************************************************
 * @section Default CoAP Handler
 **************************************************************************************************/

static void coap_default_handler(void *p_context, otCoapHeader *p_header, otMessage *p_message,
                                 const otMessageInfo *p_message_info)
{
    (void)p_context;
    (void)p_header;
    (void)p_message;
    (void)p_message_info;

    NRF_LOG_INFO("Received CoAP message that does not match any request or resource\r\n");
}

/***************************************************************************************************
 * @section State
 **************************************************************************************************/

static void role_change_handler(void * p_context, otDeviceRole role)
{
    (void)p_context;

    switch(role)
    {
        case OT_DEVICE_ROLE_CHILD:
        case OT_DEVICE_ROLE_ROUTER:
        case OT_DEVICE_ROLE_LEADER:
            break;

        case OT_DEVICE_ROLE_DISABLED:
        case OT_DEVICE_ROLE_DETACHED:
        default:
            m_app.peer_address = m_unspecified_ipv6;
            break;
    }
}

static void state_changed_callback(uint32_t flags, void * p_context)
{
    if (flags & OT_CHANGED_THREAD_ROLE)
    {
        role_change_handler(p_context, otThreadGetDeviceRole(p_context));
    }

    if (flags & OT_CHANGED_THREAD_PARTITION_ID)
    {
        m_app.peer_address = m_unspecified_ipv6;
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
            if (!otIp6IsAddressEqual(&m_app.peer_address, &m_unspecified_ipv6))
            {
                unicast_light_request_send(m_app.p_ot_instance, LIGHT_TOGGLE);
                otLinkSetPollPeriod(m_app.p_ot_instance, COAP_POLL_PERIOD);
            }
            break;

        case BSP_EVENT_KEY_1:
            m_app.multicast_light_on = !m_app.multicast_light_on;
            if (m_app.multicast_light_on)
            {
                multicast_light_request_send(m_app.p_ot_instance, LIGHT_ON);
            }
            else
            {
                multicast_light_request_send(m_app.p_ot_instance, LIGHT_OFF);
            }
            break;

        case BSP_EVENT_KEY_2:
            {
                otLinkModeConfig mode = otThreadGetLinkMode(m_app.p_ot_instance);

                if (mode.mRxOnWhenIdle)
                {
                    mode.mRxOnWhenIdle = false;
                    LEDS_OFF(BSP_LED_2_MASK);
                }
                else
                {
                    mode.mRxOnWhenIdle = true;
                    LEDS_ON(BSP_LED_2_MASK);
                }

                assert(otThreadSetLinkMode(m_app.p_ot_instance, mode) == OT_ERROR_NONE);
            }
            break;

        case BSP_EVENT_KEY_3:
            provisioning_request_send(m_app.p_ot_instance);
            otLinkSetPollPeriod(m_app.p_ot_instance, COAP_POLL_PERIOD);
            break;

        default:
            return; // no implementation needed
    }
}

/***************************************************************************************************
 * @section Initialization
 **************************************************************************************************/

static void thread_init(void)
{
    otInstance *p_instance;

    PlatformInit(0, NULL);

    p_instance = otInstanceInit();
    assert(p_instance);

    NRF_LOG_INFO("Thread version: %s\r\n", (uint32_t)otGetVersionString());
    NRF_LOG_INFO("Network name:   %s\r\n", (uint32_t)otThreadGetNetworkName(p_instance));

    assert(otSetStateChangedCallback(p_instance, &state_changed_callback, p_instance) == OT_ERROR_NONE);

    if (!otDatasetIsCommissioned(p_instance))
    {
        assert(otLinkSetChannel(p_instance, THREAD_CHANNEL) == OT_ERROR_NONE);
        assert(otLinkSetPanId(p_instance, THREAD_PANID) == OT_ERROR_NONE);
    }

    otLinkModeConfig mode;
    memset(&mode, 0, sizeof(mode));
    mode.mRxOnWhenIdle       = false; // Join network as SED.
    mode.mSecureDataRequests = true;
    assert(otThreadSetLinkMode(p_instance, mode) == OT_ERROR_NONE);

    otLinkSetPollPeriod(p_instance, DEFAULT_POLL_PERIOND);
    otThreadSetChildTimeout(p_instance, DEFAULT_CHILD_TIMEOUT);

    assert(otIp6SetEnabled(p_instance, true) == OT_ERROR_NONE);
    assert(otThreadSetEnabled(p_instance, true) == OT_ERROR_NONE);

    m_app.p_ot_instance = p_instance;
}

static void coap_init()
{
    assert(otCoapStart(m_app.p_ot_instance, OT_DEFAULT_COAP_PORT) == OT_ERROR_NONE);
    otCoapSetDefaultHandler(m_app.p_ot_instance, coap_default_handler, NULL);
}

static void timer_init(void)
{
    uint32_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

static void leds_init(void)
{
    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);
}

static void thread_bsp_init(void)
{
    uint32_t err_code;
    err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_thread_init(m_app.p_ot_instance);
    APP_ERROR_CHECK(err_code);
}

/***************************************************************************************************
 * @section Main
 **************************************************************************************************/

int main(int argc, char *argv[])
{
    NRF_LOG_INIT(NULL);

    thread_init();
    coap_init();

    timer_init();
    thread_bsp_init();
    leds_init();

    while (true)
    {
        otTaskletsProcess(m_app.p_ot_instance);
        PlatformProcessDrivers(m_app.p_ot_instance);

        if (!otTaskletsArePending(m_app.p_ot_instance))
        {
            // Enter sleep state if no more tasks are pending.
            __WFE();
        }
    }
}

/**
 *@}
 **/

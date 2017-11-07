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
 * @defgroup cloud_coap_client_example_main main.c
 * @{
 * @ingroup cloud_coap_client_example_example
 * @brief Simple Cloud CoAP Client Example Application main file.
 *
 * @details This example demonstrates a CoAP client application that sends emulated
 *          temperature value to the thethings.io cloud. Example uses NAT64 on the
 *          Nordic's Thread Border Router soulution for IPv4 connectivity.
 *
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
#include <openthread/cli.h>
#include <openthread/dns.h>
#include <openthread/platform/platform.h>
#include <openthread/platform/alarm.h>

#define LED_INTERVAL              200

#define CLOUD_HOSTNAME            "coap.thethings.io"            /**< Hostname of the thethings.io cloud. */
#define DNS_SERVER_IP             "fd00:64:123:4567::1"          /**< IPv6 of the DNS server (DNS64). */

#define CLOUD_URI_PATH            "v2/things/{THING-TOKEN}"      /**< Put your things URI here. */
#define CLOUD_THING_RESOURCE      "temp"                         /**< Thing resource name. */
#define CLOUD_COAP_CONTENT_FORMAT 50                             /**< Use application/json content format type. */

#define TEMPERATURE_INIT          22                             /**< The initial value of temperature. */
#define TEMPERATURE_MIN           15                             /**< Minimal possible temperature value. */
#define TEMPERATURE_MAX           30                             /**< Maximum possible temperature value. */

typedef enum
{
    APP_STATE_IDLE = 0,            /**< Application in IDLE state. */
    APP_STATE_HOSTNAME_RESOLVING,  /**< Application is currently resolving cloud hostname. */
    APP_STATE_RUNNING              /**< Application is running. */
} state_t;

typedef struct
{
    otInstance   * p_ot_instance;   /**< A pointer to the OpenThread instance. */
    otIp6Address   cloud_address;   /**< NAT64 address of the thethings.io cloud. */
    uint16_t       temperature;     /**< The value of emulated temperature. */
    state_t        state;           /**< The currect state of application. */
} application_t;

application_t m_app =
{
    .p_ot_instance = NULL,
    .cloud_address = { .mFields.m8 = { 0 } },
    .temperature   = TEMPERATURE_INIT,
    .state         = APP_STATE_IDLE
};

static const char m_cloud_hostname[] = CLOUD_HOSTNAME;
static const otIp6Address m_unspecified_ipv6 = { .mFields.m8 = { 0 } };

/***************************************************************************************************
 * @section DNS Client
 **************************************************************************************************/

static void dns_response_handler(void * p_context, const char * p_hostname, otIp6Address * p_address,
                                 uint32_t ttl, otError error)
{
    (void)p_context;

    if (m_app.state != APP_STATE_HOSTNAME_RESOLVING)
    {
        return;
    }

    m_app.state = APP_STATE_RUNNING;

    if (error != OT_ERROR_NONE)
    {
        NRF_LOG_INFO("DNS response error %d.\r\n", error);
        return;
    }

    // Check if its hostname of interest.
    if (p_hostname == m_cloud_hostname && ttl > 0)
    {
        m_app.cloud_address = *p_address;
        m_app.state         = APP_STATE_RUNNING;
    }
}

static void hostname_resolve(otInstance * p_instance,
                             const char * p_hostname)
{
    otError       error;
    otDnsQuery    query;
    otMessageInfo message_info;

    memset(&message_info, 0, sizeof(message_info));
    message_info.mInterfaceId = OT_NETIF_INTERFACE_ID_THREAD;
    message_info.mPeerPort    = OT_DNS_DEFAULT_DNS_SERVER_PORT;
    otIp6AddressFromString(DNS_SERVER_IP, &message_info.mPeerAddr);

    query.mHostname    = m_cloud_hostname;
    query.mMessageInfo = &message_info;
    query.mNoRecursion = false;

    error = otDnsClientQuery(p_instance, &query, dns_response_handler, NULL);
    if (error != OT_ERROR_NONE)
    {
        NRF_LOG_INFO("Failed to perform DNS Query.\r\n");
    }
}

/***************************************************************************************************
 * @section Cloud CoAP
 **************************************************************************************************/

 static void cloud_data_send(otInstance * p_instance,
                             const char * p_uri_path,
                             char       * p_payload)
 {
     otError      error = OT_ERROR_NO_BUFS;
     otCoapHeader header;
     otCoapOption content_format_option;
     otMessage * p_request;
     otMessageInfo message_info;
     uint8_t content_format = CLOUD_COAP_CONTENT_FORMAT;

     do
     {
         content_format_option.mNumber = OT_COAP_OPTION_CONTENT_FORMAT;
         content_format_option.mLength = 1;
         content_format_option.mValue  = &content_format;

         otCoapHeaderInit(&header, OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_POST);
         otCoapHeaderAppendUriPathOptions(&header, p_uri_path);
         otCoapHeaderAppendOption(&header, &content_format_option);
         otCoapHeaderSetPayloadMarker(&header);

         p_request = otCoapNewMessage(p_instance, &header);
         if (p_request == NULL)
         {
             NRF_LOG_INFO("Failed to allocate message for CoAP Request\r\n");
             break;
         }

         error = otMessageAppend(p_request, p_payload, strlen(p_payload));
         if (error != OT_ERROR_NONE)
         {
             break;
         }

         memset(&message_info, 0, sizeof(message_info));
         message_info.mInterfaceId = OT_NETIF_INTERFACE_ID_THREAD;
         message_info.mPeerPort = OT_DEFAULT_COAP_PORT;
         message_info.mPeerAddr = m_app.cloud_address;

         error = otCoapSendRequest(p_instance, p_request, &message_info, NULL, NULL);

     } while (false);

     if (error != OT_ERROR_NONE && p_request != NULL)
     {
         NRF_LOG_INFO("Failed to send CoAP Request: %d\r\n", error);
         otMessageFree(p_request);
     }
 }

 static void cloud_data_update(void)
 {
     char payload_buffer[64];

     sprintf(payload_buffer,
             "{\"values\":[{\"key\":\"%s\",\"value\":\"%d\"}]}",
             CLOUD_THING_RESOURCE, m_app.temperature);

     cloud_data_send(m_app.p_ot_instance, CLOUD_URI_PATH, payload_buffer);
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

static void handle_role_change(void * p_context, otDeviceRole role)
{
    switch(role)
    {
        case OT_DEVICE_ROLE_CHILD:
        case OT_DEVICE_ROLE_ROUTER:
        case OT_DEVICE_ROLE_LEADER:
            if (m_app.state == APP_STATE_IDLE)
            {
                m_app.state = APP_STATE_HOSTNAME_RESOLVING;
                hostname_resolve(p_context, m_cloud_hostname);
            }
            break;

        case OT_DEVICE_ROLE_DISABLED:
        case OT_DEVICE_ROLE_DETACHED:
        default:
            m_app.state         = APP_STATE_IDLE;
            m_app.cloud_address = m_unspecified_ipv6;
            break;
    }
}

static void state_changed_callback(uint32_t flags, void * p_context)
{
    if (flags & OT_CHANGED_THREAD_ROLE)
    {
        handle_role_change(p_context, otThreadGetDeviceRole(p_context));
    }

    NRF_LOG_INFO("State changed! Flags: 0x%08x Current role: %d\r\n", flags, otThreadGetDeviceRole(p_context));
}

/***************************************************************************************************
 * @section Buttons
 **************************************************************************************************/

static void bsp_event_handler(bsp_event_t event)
{
    // Check if cloud hostname has been resolved.
    if (m_app.state != APP_STATE_RUNNING)
    {
        return;
    }

    // If IPv6 address of the cloud is unknown try to resolve hostname.
    if (otIp6IsAddressEqual(&m_app.cloud_address, &m_unspecified_ipv6))
    {
        hostname_resolve(m_app.p_ot_instance, m_cloud_hostname);
        return;
    }

    switch (event)
    {
        case BSP_EVENT_KEY_0:
            if (m_app.temperature == TEMPERATURE_MIN)
            {
                // The minimal temperature has been already reached.
                break;
            }

            // Decrement temperature value.
            m_app.temperature -= 1;

            cloud_data_update();
            break;

        case BSP_EVENT_KEY_1:
            if (m_app.temperature == TEMPERATURE_MAX)
            {
                // The maximum temperature has been already reached.
                break;
            }

            // Increment temperature value.
            m_app.temperature += 1;

            cloud_data_update();
            break;

        case BSP_EVENT_KEY_2:
            break;

        case BSP_EVENT_KEY_3:
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
    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, bsp_event_handler);
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
    }
}

/**
 *@}
 **/

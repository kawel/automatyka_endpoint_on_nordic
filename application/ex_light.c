/**
 * @file ex_light.c
 * @author Pawel Radecki
 * @date 26 Nov 2017
 *
 */

#include "ex_light.h"

#include "nrf_log.h"

#include <openthread/coap.h>


static const otIp6Address m_unspecified_ipv6 = { .mFields.m8 = { 0 } };

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


static void unicast_light_request_send(otInstance * p_instance, uint8_t command);
static void multicast_light_request_send(otInstance * p_instance, uint8_t command);
static void provisioning_request_send(otInstance * p_instance);




void ex_light_init(otInstance *ot_instance)
{
    m_app.p_ot_instance = ot_instance;
}


void ex_light_set_peer_address_unspecified(void)
{
    m_app.peer_address = m_unspecified_ipv6;
}


void ex_light_unicast(void)
{
    if (!otIp6IsAddressEqual(&m_app.peer_address, &m_unspecified_ipv6))
    {
        unicast_light_request_send(m_app.p_ot_instance, LIGHT_TOGGLE);
    }
}


void ex_light_multicast(void)
{
    m_app.multicast_light_on = !m_app.multicast_light_on;
    if (m_app.multicast_light_on)
    {
        multicast_light_request_send(m_app.p_ot_instance, LIGHT_ON);
    }
    else
    {
        multicast_light_request_send(m_app.p_ot_instance, LIGHT_OFF);
    }
}


void ex_light_provisioning(void)
{

    provisioning_request_send(m_app.p_ot_instance);
}


void thread_command_handler(const uint8_t * p_command_str, uint16_t length)
{
    if (strncmp(COMMAND_REQUEST_UNICAST, (char *)p_command_str, strlen(COMMAND_REQUEST_UNICAST)) == 0)
    {
        if (!otIp6IsAddressEqual(&m_app.peer_address, &m_unspecified_ipv6))
        {
            unicast_light_request_send(m_app.p_ot_instance, LIGHT_TOGGLE);
        }
    }
    else if (strncmp(COMMAND_REQUEST_MULTICAST, (char *)p_command_str, strlen(COMMAND_REQUEST_MULTICAST)) == 0)
    {
        m_app.multicast_light_on = !m_app.multicast_light_on;
        if (m_app.multicast_light_on)
        {
            multicast_light_request_send(m_app.p_ot_instance, LIGHT_ON);
        }
        else
        {
            multicast_light_request_send(m_app.p_ot_instance, LIGHT_OFF);
        }
    }
    else if (strncmp(COMMAND_REQUEST_PROVISIONING, (char *)p_command_str, strlen(COMMAND_REQUEST_PROVISIONING)) == 0)
    {
         provisioning_request_send(m_app.p_ot_instance);
    }
    else
    {
        // Do nothing.
    }
}


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

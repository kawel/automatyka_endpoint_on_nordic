/**
 * @file ep_coap.c
 * @author Pawel Radecki
 * @date 13 Nov 2017
 *
 */

#include "ep_coap.h"

#include <string.h>
#include <stdio.h>

#include "utils.h"
//#include "version.h" // TODO: not present right now stub it

#include "nrf_log.h"
#include "boards.h"

#include <openthread/coap.h>


static void req_hdl_output(void *aContext, otCoapHeader *aHeader, otMessage *aMessage, const otMessageInfo *aMessageInfo);
static void req_hdl_input(void *aContext, otCoapHeader *aHeader, otMessage *aMessage, const otMessageInfo *aMessageInfo);
static void req_hdl_status(void *aContext, otCoapHeader *aHeader, otMessage *aMessage, const otMessageInfo *aMessageInfo);
static void req_hdl_version(void *aContext, otCoapHeader *aHeader, otMessage *aMessage, const otMessageInfo *aMessageInfo);

static void send_response(void *aContext, otCoapHeader *aHeader, const otMessageInfo *aMessageInfo, char *respContent, size_t respLen);

// Outputs which can be controlled by USER.
// Available methods:
// POST
// GET
static otCoapResource res_output = {
    .mUriPath = "output",
    .mHandler = req_hdl_output,
};

// Inputs which can be read by USER.
// Available methods:
// GET
static otCoapResource res_input = {
    .mUriPath = "input",
    .mHandler = req_hdl_input,
};

// Status of the device which can be read by USER.
// Available methods:
// GET
static otCoapResource res_status = {
    .mUriPath = "status",
    .mHandler = req_hdl_status,
};

// Version of the SW which can be read by USER.
// Available methods:
// GET
static otCoapResource res_version = {
    .mUriPath = "version",
    .mHandler = req_hdl_version,
};

int ep_coap_init(otInstance *aInstance)
{
    NRF_LOG_INFO("EP CoAP: add resources and start server\r\n");

    // add resources to the server
    res_output.mContext = aInstance;
    SuccessOrExit(otCoapAddResource(aInstance, &res_output));
    NRF_LOG_INFO("EP CoAP: output added\r\n");

    res_input.mContext = aInstance;
    SuccessOrExit(otCoapAddResource(aInstance, &res_input));
    NRF_LOG_INFO("EP CoAP: input added\r\n");

    res_status.mContext = aInstance;
    SuccessOrExit(otCoapAddResource(aInstance, &res_status));
    NRF_LOG_INFO("EP CoAP: status added\r\n");

    res_version.mContext = aInstance;
    SuccessOrExit(otCoapAddResource(aInstance, &res_version));
    NRF_LOG_INFO("EP CoAP: version added\r\n");

    // start server
    SuccessOrExit(otCoapStart(aInstance, OT_DEFAULT_COAP_PORT));
    NRF_LOG_INFO("EP CoAP: Started\r\n");

    return 0;

exit:
    NRF_LOG_INFO("EP CoAP: Failed\r\n");
    return -1;
}


void req_hdl_output(void *aContext, otCoapHeader *aHeader, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
    uint8_t led_state = 0;

    NRF_LOG_INFO("EP resources: output callback\r\n");

    switch (otCoapHeaderGetCode(aHeader))
    {
        case OT_COAP_CODE_GET:
            led_state = (bsp_board_led_state_get(BSP_BOARD_LED_3) << 1)
                       | bsp_board_led_state_get(BSP_BOARD_LED_1);
            break;

        case OT_COAP_CODE_DELETE:
            break;

        case OT_COAP_CODE_PUT:
            break;

        case OT_COAP_CODE_POST:
        {
            uint16_t length = otMessageGetLength(aMessage) - otMessageGetOffset(aMessage);
            if (length == 1)
            {
                otMessageRead(aMessage, otMessageGetOffset(aMessage), &led_state, length);

                if (led_state & 1)
                {
                    bsp_board_led_on(BSP_BOARD_LED_1);
                }
                else
                {
                    bsp_board_led_off(BSP_BOARD_LED_1);
                }

                if (led_state & 2)
                {
                    bsp_board_led_on(BSP_BOARD_LED_3);
                }
                else
                {
                    bsp_board_led_off(BSP_BOARD_LED_3);
                }
            }
        }
            break;

        default:
            //mInterpreter.mServer->OutputFormat("Undefined\r\n");
            return;
    }

    send_response(aContext, aHeader, aMessageInfo, (char *)&led_state, sizeof(led_state));
}

// Only GET method is available. Currently in this sample code reads state of the LED pins.
// But in contrast to /output return state as ASCII text.
static void req_hdl_input(void *aContext, otCoapHeader *aHeader, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
    char input_state[25] = "";
    (void)aMessage;

    NRF_LOG_INFO("EP resources: input callback\r\n");

    switch (otCoapHeaderGetCode(aHeader))
    {
        case OT_COAP_CODE_GET:
            sprintf(input_state, "LED_0: %s, LED_1: %s", bsp_board_led_state_get(BSP_BOARD_LED_1) == true ? "On" : "Off",
                                                         bsp_board_led_state_get(BSP_BOARD_LED_3) == true ? "On" : "Off");
            break;

        case OT_COAP_CODE_DELETE:
            break;

        case OT_COAP_CODE_PUT:
            break;

        case OT_COAP_CODE_POST:
            break;

        default:
            return;
    }

    send_response(aContext, aHeader, aMessageInfo, input_state, strlen(input_state));
}

static void req_hdl_status(void *aContext, otCoapHeader *aHeader, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
    char status[] = "Everything is fine";
    (void)aMessage;

    NRF_LOG_INFO("EP resources: status callback\r\n");

    switch (otCoapHeaderGetCode(aHeader))
    {
        case OT_COAP_CODE_GET:
            break;

        case OT_COAP_CODE_DELETE:
            break;

        case OT_COAP_CODE_PUT:
            break;

        case OT_COAP_CODE_POST:
            break;

        default:
            return;
    }

    send_response(aContext, aHeader, aMessageInfo, status, strlen(status));
}

static void prepare_version_data(char *buffer)
{
    static char *version_data = "prototyp 1234abc Nov 13 2017 04:48:23";

    // TODO: Add Version module and activate using it.
    memcpy(buffer, version_data, strlen(version_data) + 1);

#if 0
    size_t len_prog_name, len_sw_rev, len_time, len_date;
    int offset = 0;

    len_prog_name = VER_get_program_name(NULL, 0);
    len_sw_rev = VER_get_sw_revision(NULL, 0);
    len_time = VER_get_compilation_time(NULL, 0);
    len_date = VER_get_compilation_date(NULL, 0);

    offset += VER_get_program_name(&buffer[offset], len_prog_name);
    buffer[offset++] = ' ';
    offset += VER_get_sw_revision(&buffer[offset], len_sw_rev);
    buffer[offset++] = ' ';
    offset += VER_get_compilation_date(&buffer[offset], len_date);
    buffer[offset++] = ' ';
    offset += VER_get_compilation_time(&buffer[offset], len_time);
    buffer[offset++] = '\0';
#endif
}

static void req_hdl_version(void *aContext, otCoapHeader *aHeader, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
    char version[50] = "";
    (void)aMessage;

    NRF_LOG_INFO("EP resources: version callback\r\n");

    switch (otCoapHeaderGetCode(aHeader))
    {
        case OT_COAP_CODE_GET:
            prepare_version_data(version);
            break;

        case OT_COAP_CODE_DELETE:
            break;

        case OT_COAP_CODE_PUT:
            break;

        case OT_COAP_CODE_POST:
            break;

        default:
            return;
    }

    send_response(aContext, aHeader, aMessageInfo, version, strlen(version));
}

static void send_response(void *aContext, otCoapHeader *aHeader, const otMessageInfo *aMessageInfo, char *respContent, size_t respLen)
{
    otError error = OT_ERROR_NONE;
    otCoapHeader responseHeader;
    otMessage *responseMessage;
    otCoapCode responseCode = OT_COAP_CODE_EMPTY;

    NRF_LOG_INFO("EP resources: send response\r\n");

    if ((otCoapHeaderGetType(aHeader) == OT_COAP_TYPE_CONFIRMABLE) || otCoapHeaderGetCode(aHeader) == OT_COAP_CODE_GET)
    {
        if (otCoapHeaderGetCode(aHeader) == OT_COAP_CODE_GET)
        {
            responseCode = OT_COAP_CODE_CONTENT;
        }
        else
        {
            responseCode = OT_COAP_CODE_VALID;
        }

        otCoapHeaderInit(&responseHeader, OT_COAP_TYPE_ACKNOWLEDGMENT, responseCode);
        otCoapHeaderSetMessageId(&responseHeader, otCoapHeaderGetMessageId(aHeader));
        otCoapHeaderSetToken(&responseHeader, otCoapHeaderGetToken(aHeader), otCoapHeaderGetTokenLength(aHeader));

        if (otCoapHeaderGetCode(aHeader) == OT_COAP_CODE_GET)
        {
            otCoapHeaderSetPayloadMarker(&responseHeader);
        }

        responseMessage = otCoapNewMessage(aContext, &responseHeader);
        VerifyOrExit(responseMessage != NULL);

        if (otCoapHeaderGetCode(aHeader) == OT_COAP_CODE_GET)
        {
            SuccessOrExit(otMessageAppend(responseMessage, respContent, respLen));
        }

        SuccessOrExit(otCoapSendResponse(aContext, responseMessage, aMessageInfo));
    }

exit:
    if (error != OT_ERROR_NONE && responseMessage != NULL)
    {
        NRF_LOG_INFO("EP resources: send response Failed\r\n");
        otMessageFree(responseMessage);
    }
    else if (responseCode >= OT_COAP_CODE_RESPONSE_MIN)
    {
        NRF_LOG_INFO("EP resources: response sent successfully\r\n");
    }
}

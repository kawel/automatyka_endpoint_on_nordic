/**
 * @file ep_udp.c
 * @author Pawel Radecki
 * @date 10 Nov 2017
 *
 */

#include "ep_udp.h"
#include "utils.h"

#include "nrf_log.h"
#include "ep_bsp.h"

#include <openthread/udp.h>


#ifdef EP_UDP_LOGGING
    #define EP_UDP_LOG_INFO(...)   NRF_LOG_INFO("EP UDP: " __VA_ARGS__)
    #define EP_UDP_LOG_ERROR(...)  NRF_LOG_ERROR("EP UDP: " __VA_ARGS__)
#else
    #define EP_UDP_LOG_INFO(...)
    #define EP_UDP_LOG_ERROR(...)
#endif


static otUdpSocket udp_socket;
static otSockAddr udp_socket_addr =
{
    .mPort = 8000
};

static void udp_receive(void *context, otMessage *message, const otMessageInfo *message_info)
{
    (void) message_info;
    (void) context;

    uint16_t payloadLength = otMessageGetLength(message) - otMessageGetOffset(message);
    char buf[1];

    EP_UDP_LOG_INFO("UDP receive callback\r\n");

    VerifyOrExit(payloadLength <= sizeof(buf));
    otMessageRead(message, otMessageGetOffset(message), buf, payloadLength);

    EP_UDP_LOG_INFO("UDP received data: %d\r\n", buf[0]);
    if (buf[0] & 1)
    {
        ep_bsp_output_1_on();
    }
    else
    {
        ep_bsp_output_1_off();
    }

    if (buf[0] & 2)
    {
        ep_bsp_output_2_on();
    }
    else
    {
        ep_bsp_output_2_off();
    }

exit:
    return;
}

int ep_udp_start(otInstance *ot_instance)
{
    SuccessOrExit(otUdpOpen(ot_instance, &udp_socket, udp_receive, NULL));
    EP_UDP_LOG_INFO("UDP Server: Open\r\n");
    SuccessOrExit(otUdpBind(&udp_socket, &udp_socket_addr));
    EP_UDP_LOG_INFO("UDP Server: Bind\r\n");
    EP_UDP_LOG_INFO("UDP Server: Started\r\n");
    return 0;

exit:
    EP_UDP_LOG_ERROR("UDP Server: Failed\r\n");
    return -1;
}



/**
 * @file ep_udp.c
 * @author Pawel Radecki
 * @date 10 Nov 2017
 *
 */

#include "ep_udp.h"
#include "utils.h"

#include "nrf_log.h"
#include "boards.h"

#include <openthread/udp.h>

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

    NRF_LOG_INFO("UDP receive callback\r\n");

    VerifyOrExit(payloadLength <= sizeof(buf));
    otMessageRead(message, otMessageGetOffset(message), buf, payloadLength);

    NRF_LOG_INFO("UDP received data: %d\r\n", buf[0]);
    if (buf[0] & 1)
    {
        bsp_board_led_on(BSP_BOARD_LED_1);
    }
    else
    {
        bsp_board_led_off(BSP_BOARD_LED_1);
    }

    if (buf[0] & 2)
    {
        bsp_board_led_on(BSP_BOARD_LED_3);
    }
    else
    {
        bsp_board_led_off(BSP_BOARD_LED_3);
    }

exit:
    return;
}

int ep_udp_start(otInstance *ot_instance)
{
    SuccessOrExit(otUdpOpen(ot_instance, &udp_socket, udp_receive, NULL));
    NRF_LOG_INFO("UDP Server: Open\r\n");
    SuccessOrExit(otUdpBind(&udp_socket, &udp_socket_addr));
    NRF_LOG_INFO("UDP Server: Bind\r\n");
    NRF_LOG_INFO("UDP Server: Started\r\n");
    return 0;

exit:
    NRF_LOG_INFO("UDP Server: Failed\r\n");
    return -1;
}


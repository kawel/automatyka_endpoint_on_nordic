/**
 * @file ep_udp.h
 * @author Pawel Radecki
 * @date 10 Nov 2017
 * @brief File defines interface to run UDP server on the End Point.
 * @warning This file is created for backward compatibility.
 *          To communicate with the EP should be used CoAP resources.
 *
 */

#ifndef EP_UDP_H_
#define EP_UDP_H_

#include <openthread/openthread.h>

#ifdef __cplusplus
extern "C" {
#endif

int ep_udp_start(otInstance *ot_instance);

#ifdef __cplusplus
}
#endif

#endif /* EP_UDP_H_ */

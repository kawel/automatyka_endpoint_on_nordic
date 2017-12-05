/**
 * @file ep_coap.h
 * @author Pawel Radecki
 * @date 13 Nov 2017
 * @brief File defines interface to register resources on the End Point.
 *
 */

#ifndef EP_COAP_H_
#define EP_COAP_H_

#include <openthread/openthread.h>

#ifdef __cplusplus
extern "C" {
#endif

int ep_coap_init(otInstance *aInstance);

#ifdef __cplusplus
}
#endif

#endif /* EP_COAP_H_ */

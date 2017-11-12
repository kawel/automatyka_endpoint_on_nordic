/**
 * @file ep_cfg.h
 * @author Pawel Radecki
 * @date 10 Nov 2017
 * @brief File defines interface to configure End Point.
 *
 */

#ifndef EP_CFG_H_
#define EP_CFG_H_

#include <openthread/openthread.h>

#ifdef __cplusplus
extern "C" {
#endif

int ep_cfg_init(void);

/* This function is waiting for the new (very first or updated) configuration
 * and when it's valid is is used to configure the EP */
int ep_cfg_check_and_apply(otInstance *ot_instance);

#ifdef __cplusplus
}
#endif

#endif /* EP_CFG_H_ */

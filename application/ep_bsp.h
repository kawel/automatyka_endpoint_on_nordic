/**
 * @file ep_bsp.h
 * @author Pawel Radecki
 * @date 11 Nov 2017
 * @brief File allows to map resource based on the used HW.
 *
 */

#ifndef EP_BSP_H_
#define EP_BSP_H_

#include <openthread/openthread.h>

// TODO: check HW version and based on it do mapping to proper peripherals
// Create interface which will be used to streamline access to the peripherals?
#if defined(BOARD_PCA10056)

#elif defined(KIWI)

#else
    #error "HW Board is not defined"
#endif

#ifdef __cplusplus
extern "C" {
#endif

void ep_bsp_init(otInstance *ot_instance);

// TODO: Add interface to allow access to HW

#ifdef _cplusplus
}
#endif

#endif /* EP_BSP_H_ */

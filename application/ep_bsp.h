/**
 * @file ep_bsp.h
 * @author Pawel Radecki
 * @date 11 Nov 2017
 * @brief File allows to map resource based on the used HW.
 *
 */

#ifndef EP_BSP_H_
#define EP_BSP_H_

#include "sdk_errors.h"
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

typedef enum
{
    EP_BSP_INDICATE_BLE_IDLE,
    EP_BSP_INDICATE_BLE_ADVERTISING,
    EP_BSP_INDICATE_BLE_CONNECTED,
    EP_BSP_INDICATE_BLE_LED_ON,
    EP_BSP_INDICATE_BLE_LED_OFF,
} ep_bsp_indication_ble_t;

typedef enum
{
    EP_BSP_INDICATE_THREAD_DISABLED,
    EP_BSP_INDICATE_THREAD_DETACHED,
    EP_BSP_INDICATE_THREAD_CONNECTED,
    EP_BSP_INDICATE_THREAD_LED_ON,
    EP_BSP_INDICATE_THREAD_LED_OFF,
} ep_bsp_indication_thread_t;

void ep_bsp_init(void);

ret_code_t ep_bsp_indication_ble_set(ep_bsp_indication_ble_t indicate);
ret_code_t ep_bsp_indication_thread_set(ep_bsp_indication_thread_t indicate);

void ep_bsp_output_1_on(void);
void ep_bsp_output_1_off(void);
bool ep_bsp_output_1_state_get(void);

void ep_bsp_output_2_on(void);
void ep_bsp_output_2_off(void);
bool ep_bsp_output_2_state_get(void);

#ifdef _cplusplus
}
#endif

#endif /* EP_BSP_H_ */

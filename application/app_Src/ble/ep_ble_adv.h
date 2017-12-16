/*
 * ep_ble_adv.h
 *
 *  Created on: 10.12.2017
 *      Author: Samsung
 */

#ifndef APP_SRC_BLE_EP_BLE_ADV_H_
#define APP_SRC_BLE_EP_BLE_ADV_H_


/**@brief Function for initializing the Advertising functionality.
 */
 void appble_adv_init(void);

/**@brief Function called when advertising is stopped.
 */
void appble_adv_stopped(void);

/**@brief Function for starting advertising.
 */
void appble_adv_start(void);

bool appble_adv_get_state(void);

void appble_adv_chng_temp(void);

#endif /* APP_SRC_BLE_EP_BLE_ADV_H_ */

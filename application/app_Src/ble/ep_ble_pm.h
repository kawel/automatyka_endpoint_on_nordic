/*
 * ep_ble_pm.h
 *
 *  Created on: 10.12.2017
 *      Author: Samsung
 */

#ifndef APP_SRC_BLE_EP_BLE_PM_H_
#define APP_SRC_BLE_EP_BLE_PM_H_


/**@brief Function for the Peer Manager initialization.
 */
void appble_pm_init(void);

void appble_pm_set_list(void);

/**@brief Clear bond information from persistent storage.
 */
void appble_pm_delete_bonds(void);

#endif /* APP_SRC_BLE_EP_BLE_PM_H_ */

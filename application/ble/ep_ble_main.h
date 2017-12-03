/**
 * @file ep_ble_main.h
 * @author Aleksander Demianowski
 * @date 2 Dec 2017
 * @brief The main BLE functionality
 *
 */

#ifndef EP_BLE_MAIN_H_
#define EP_BLE_MAIN_H_

#define BLE_LOGGING     // If defined BLE logging information is turned on

void appble_init(void(*disp_fun)(uint32_t));
void appble_start(bool erase_bonds);
void appble_adv_triggered(void);
void appble_disconn_triggered(void);
void appble_wl_off_triggered(void);
void appble_sys_evt_dispatch(uint32_t sys_evt);

#endif //EP_BLE_MAIN_H_


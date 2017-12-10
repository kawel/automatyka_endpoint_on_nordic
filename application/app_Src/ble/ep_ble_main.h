/**
 * @file ep_ble_main.h
 * @author Aleksander Demianowski
 * @date 2 Dec 2017
 * @brief The main BLE functionality
 *
 */

#ifndef EP_BLE_MAIN_H_
#define EP_BLE_MAIN_H_

#include "config/app_config.h"

#define CONN_CFG_TAG                    1                                           /**< A tag that refers to the BLE stack configuration we set with @ref sd_ble_cfg_set. Default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */

#ifdef BLE_LOGGING
       #define EP_BLE_LOG_INFO( ... )      do{      \
											 	 NRF_LOG_INFO("EP BLE: " __VA_ARGS__); \
											 }while(0)
#else
       #define EP_BLE_LOG_INFO( ... )      do{      \
										 	 }while(0)
#endif //BLE_LOGGING

void appble_init(void(*disp_fun)(uint32_t));
void appble_start(bool erase_bonds);
void appble_adv_triggered(void);
void appble_disconn_triggered(void);
void appble_wl_off_triggered(void);
void appble_sys_evt_dispatch(uint32_t sys_evt);
bool appble_get_conn_state(void);
#endif //EP_BLE_MAIN_H_


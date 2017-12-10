/*
 * ep_ble_adv.c
 *
 *  Created on: 10.12.2017
 *      Author: aledem
 */


#include <stdint.h>
#include <string.h>
#include <assert.h>

#include "nordic_common.h"
#include "nrf.h"
#include "softdevice_handler.h"
#include "nrf_ble_gatt.h"
#include "app_timer.h"
#include "app_button.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
//services
#include "ble_lbs.h"
#include "ble_nus.h"

#include "ep_ble_adv.h"
#include "ep_ble_main.h"
#include "ep_ble_pm.h"
#include "ep_bsp.h"


#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                200                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      30                                          /**< The advertising timeout (in units of seconds). */

//#define DEVICE_NAME_DEFAULT             "SmartConnect_defalut"                      /**< Name of device. Will be included in the advertising data. */
//#define DEVICE_NAME_BUTTON              "SmartConnect_button_"                      /**< Name of device. Will be included in the advertising data. */

static bool                             adv_state_on;                               /**< Status indicating that advertising is turned on. */


static void on_adv_evt(ble_adv_evt_t ble_adv_evt);






/**@brief Function for initializing the Advertising functionality.
 */
void appble_adv_init(void)
{
    uint32_t               err_code;
    ble_advdata_t          advdata;
    ble_advdata_t          scanrsp;
    ble_adv_modes_config_t options;
    ble_uuid_t      	   m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(CONN_CFG_TAG);
}


/**@brief Function called when advertising is stopped.
 */
void appble_adv_stopped(void)
{
	bool conn_state_on = appble_get_conn_state();

	EP_BLE_LOG_INFO("Advertising end\r\n");
	if (conn_state_on == false)
	{
	    ep_bsp_indication_ble_set(EP_BSP_INDICATE_BLE_IDLE);
	}
	adv_state_on = false;
}

/**@brief Function for starting advertising.
 */
void appble_adv_start(void)
{
	ret_code_t ret;

	appble_pm_set_list();

	ret = ble_advertising_start(BLE_ADV_MODE_FAST);
	APP_ERROR_CHECK(ret);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
        	EP_BLE_LOG_INFO("Advertising start\r\n");
        	adv_state_on = true;
            err_code = ep_bsp_indication_ble_set(EP_BSP_INDICATE_BLE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            EP_BLE_LOG_INFO("Advertising Idle\r\n");
            appble_adv_stopped();
//            err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
//            APP_ERROR_CHECK(err_code);
            break;
        default:
            break;
    }
}

bool appble_adv_get_state(void)
{
	return adv_state_on;
}

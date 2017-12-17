/*
 * ep_ble_adv.c
 *
 *  Created on: 10.12.2017
 *      Author: Aleksnader Demianowski
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
#include "app_timer.h"
//services
#include "ble_lbs.h"
#include "ble_nus.h"

#include "ep_ble_adv.h"
#include "ep_ble_main.h"
#include "ep_ble_pm.h"
#include "ep_bsp.h"


#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                200                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                           /**< The advertising timeout (in units of seconds). */

#define ADVETISING_EXPIRY_TIME          60000									    /**< [ms] Period when advertising is change as a reaction for button pressing. */
APP_TIMER_DEF(m_adv_chng_timer);

static uint8_t advManufMsgDefault[] = {'d', 'e', 'f', 'a', 'u', 'l', 't'};          /**< Dafault message in the manufacturer part of advertising  */
static uint8_t advManufMsgButton[]  = {'b', 'u', 't', 't', 'o', 'n', ' '};          /**< Temporary, for connection, message in the manufacturer part of advertising  */

static bool                             adv_state_on;                               /**< Status indicating that advertising is turned on. */


static void on_adv_evt(ble_adv_evt_t ble_adv_evt);
static void adv_init(uint8_t* manufacDataPtr, uint16_t manufacDataSize);

/**@brief Function for the Advertising-Change-Timer timeout handling.
 *
 * @details When timeout occurs, change advertising to the default state.
 */
static void adv_chng_timer_handler(void * p_context)
{
	EP_BLE_LOG_INFO("Timer end\r\n");
	sd_ble_gap_adv_stop();
	adv_init(advManufMsgDefault, sizeof(advManufMsgDefault));
	appble_adv_start();
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_adv_chng_timer, APP_TIMER_MODE_SINGLE_SHOT, adv_chng_timer_handler);
    APP_ERROR_CHECK(err_code);
}

static void adv_init(uint8_t* manufacDataPtr, uint16_t manufacDataSize)
{
    uint32_t               err_code;
    ble_advdata_t          advdata;
    ble_advdata_t          scanrsp;
    ble_advdata_manuf_data_t manuf_data;
    ble_adv_modes_config_t options;
    ble_uuid_t      	   m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */

    manuf_data.company_identifier = 11;   // accidental value is used
    manuf_data.data.size = manufacDataSize;
    manuf_data.data.p_data = manufacDataPtr;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.p_manuf_specific_data = &manuf_data;

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


/**@brief Function for initializing the Advertising functionality.
 */
void appble_adv_init(void)
{
	timers_init();
	adv_init(advManufMsgDefault, sizeof(advManufMsgDefault));
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

/**@brief Function called to change advertising message.
 */
void appble_adv_chng_msg(void)
{
	uint32_t err_code;

	EP_BLE_LOG_INFO("Advertising chng to Button\r\n");
	sd_ble_gap_adv_stop();
	adv_init(advManufMsgButton, sizeof(advManufMsgButton));
	appble_adv_start();

	err_code = app_timer_start(m_adv_chng_timer,
							   APP_TIMER_TICKS(ADVETISING_EXPIRY_TIME),
							   NULL);
	APP_ERROR_CHECK(err_code);
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

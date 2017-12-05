/**
 * @file ep_bsp.c
 * @author Pawel Radecki
 * @date 12 Nov 2017
 *
 */

#include "ep_bsp.h"
#include "utils.h"

#include "bsp.h"
#include "nrf_log.h"
#include "boards.h"

#include "app_timer.h"


#define EP_BSP_LED_BLE             BSP_BOARD_LED_2
#define EP_BSP_LED_THREAD          BSP_BOARD_LED_3
#define EP_BSP_OUTPUT_1            BSP_BOARD_LED_0
#define EP_BSP_OUTPUT_2            BSP_BOARD_LED_1

#define LED_ON_INTERVAL_SLOW       400
#define LED_OFF_INTERVAL_SLOW      2000

#define LED_ON_INTERVAL_MEDIUM     200
#define LED_OFF_INTERVAL_MEDIUM    800

#define LED_ON_INTERVAL_FAST       200
#define LED_OFF_INTERVAL_FAST      200

typedef enum
{
    LED_INTERVAL_SLOW,
    LED_INTERVAL_MEDIUM,
    LED_INTERVAL_FAST,
    LED_INTERVAL_CNT
} led_interval_t;

typedef struct
{
    uint16_t on_time;
    uint16_t off_time;
} led_interval_cfg_t;

static const led_interval_cfg_t led_interval_cfg[LED_INTERVAL_CNT] = {
    { LED_ON_INTERVAL_SLOW  , LED_OFF_INTERVAL_SLOW   },
    { LED_ON_INTERVAL_MEDIUM, LED_OFF_INTERVAL_MEDIUM },
    { LED_ON_INTERVAL_FAST  , LED_OFF_INTERVAL_FAST   },
};

APP_TIMER_DEF(leds_timer_ble);
APP_TIMER_DEF(leds_timer_thread);

static volatile ep_bsp_indication_ble_t ble_state;
static volatile ep_bsp_indication_thread_t thread_state;

static void led_timer_handler_ble(void *p_context);
static uint32_t bsp_led_indication_ble(ep_bsp_indication_ble_t indicate);
static uint32_t led_indication_ble(led_interval_t led_interval);

static void led_timer_handler_thread(void *p_context);
static uint32_t bsp_led_indication_thread(ep_bsp_indication_thread_t indicate);
static uint32_t led_indication_thread(led_interval_t led_interval);

static void leds_init(void);


static void led_timer_handler_ble(void *p_context)
{
    UNUSED_PARAMETER(p_context);
    UNUSED_VARIABLE(bsp_led_indication_ble(ble_state));
}

static uint32_t bsp_led_indication_ble(ep_bsp_indication_ble_t indicate)
{
    uint32_t err_code   = NRF_SUCCESS;

    ble_state = indicate;

    switch (indicate)
    {
        case EP_BSP_INDICATE_BLE_IDLE:
            err_code = led_indication_ble(LED_INTERVAL_SLOW);
            break;

        case EP_BSP_INDICATE_BLE_ADVERTISING:
            err_code = led_indication_ble(LED_INTERVAL_FAST);
            break;

        case EP_BSP_INDICATE_BLE_CONNECTED:
            err_code = led_indication_ble(LED_INTERVAL_MEDIUM);
            break;

        case EP_BSP_INDICATE_BLE_LED_ON:
            bsp_board_led_on(EP_BSP_LED_BLE);
            break;

        case EP_BSP_INDICATE_BLE_LED_OFF:
            bsp_board_led_off(EP_BSP_LED_BLE);
            break;

        default:
            break;
    }

    return err_code;
}

static uint32_t led_indication_ble(led_interval_t led_interval)
{
    uint32_t err_code = NRF_SUCCESS;
    uint32_t next_delay = 0;

    if (bsp_board_led_state_get(EP_BSP_LED_BLE))
    {
       bsp_board_led_off(EP_BSP_LED_BLE);
       next_delay = led_interval_cfg[led_interval].off_time;
    }
    else
    {
       bsp_board_led_on(EP_BSP_LED_BLE);
       next_delay = led_interval_cfg[led_interval].on_time;
    }

    err_code = app_timer_start(leds_timer_ble,
                              APP_TIMER_TICKS(next_delay),
                              NULL);

    return err_code;
}

static void led_timer_handler_thread(void *p_context)
{
    UNUSED_PARAMETER(p_context);
    UNUSED_VARIABLE(bsp_led_indication_thread(thread_state));
}

static uint32_t bsp_led_indication_thread(ep_bsp_indication_thread_t indicate)
{
    uint32_t err_code   = NRF_SUCCESS;

    thread_state = indicate;

    switch (indicate)
    {
        case EP_BSP_INDICATE_THREAD_DISABLED:
            err_code = led_indication_thread(LED_INTERVAL_SLOW);
            break;

        case EP_BSP_INDICATE_THREAD_DETACHED:
            err_code = led_indication_thread(LED_INTERVAL_FAST);
            break;

        case EP_BSP_INDICATE_THREAD_CONNECTED:
            err_code = led_indication_thread(LED_INTERVAL_MEDIUM);
            break;

        case EP_BSP_INDICATE_THREAD_LED_ON:
            bsp_board_led_on(EP_BSP_LED_THREAD);
            break;

        case EP_BSP_INDICATE_THREAD_LED_OFF:
            bsp_board_led_off(EP_BSP_LED_THREAD);
            break;

        default:
            break;
    }

    return err_code;
}

static uint32_t led_indication_thread(led_interval_t led_interval)
{
    uint32_t err_code = NRF_SUCCESS;
    uint32_t next_delay = 0;

    if (bsp_board_led_state_get(EP_BSP_LED_THREAD))
    {
       bsp_board_led_off(EP_BSP_LED_THREAD);
       next_delay = led_interval_cfg[led_interval].off_time;
    }
    else
    {
       bsp_board_led_on(EP_BSP_LED_THREAD);
       next_delay = led_interval_cfg[led_interval].on_time;
    }

    err_code = app_timer_start(leds_timer_thread,
                              APP_TIMER_TICKS(next_delay),
                              NULL);

    return err_code;
}


void ep_bsp_init(void)
{
    uint32_t err_code = NRF_SUCCESS;

    ble_state = EP_BSP_INDICATE_BLE_IDLE;
    thread_state = EP_BSP_INDICATE_THREAD_DISABLED;

    leds_init();

    err_code = app_timer_create(&leds_timer_ble,
                                   APP_TIMER_MODE_SINGLE_SHOT,
                                   led_timer_handler_ble);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&leds_timer_thread,
                               APP_TIMER_MODE_SINGLE_SHOT,
                               led_timer_handler_thread);
    APP_ERROR_CHECK(err_code);
}

static void leds_init(void)
{
    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);
}


ret_code_t ep_bsp_indication_ble_set(ep_bsp_indication_ble_t indicate)
{
    return bsp_led_indication_ble(indicate);
}

ret_code_t ep_bsp_indication_thread_set(ep_bsp_indication_thread_t indicate)
{
    return bsp_led_indication_thread(indicate);
}


void ep_bsp_output_1_on(void)
{
    bsp_board_led_on(EP_BSP_OUTPUT_1);
}

void ep_bsp_output_1_off(void)
{
    bsp_board_led_off(EP_BSP_OUTPUT_1);
}

bool ep_bsp_output_1_state_get(void)
{
   return bsp_board_led_state_get(EP_BSP_OUTPUT_1);
}

void ep_bsp_output_2_on(void)
{
    bsp_board_led_on(EP_BSP_OUTPUT_2);
}

void ep_bsp_output_2_off(void)
{
    bsp_board_led_off(EP_BSP_OUTPUT_2);
}

bool ep_bsp_output_2_state_get(void)
{
   return bsp_board_led_state_get(EP_BSP_OUTPUT_2);
}

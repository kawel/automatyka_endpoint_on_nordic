/**
 * @file app_config.h
 * @author Pawel Radecki
 * @date 05 Dec 2017
 * @brief File defines application specific configuration options.
 *
 */

#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

/*----------------------------------------------------------------------------*/
/* Logging configuration                                                      */
/*----------------------------------------------------------------------------*/
#define NRF_LOG_ENABLED  1

// <0=> Off
// <1=> Error
// <2=> Warning
// <3=> Info
// <4=> Debug
#define NRF_LOG_DEFAULT_LEVEL 3

#define BLE_LOGGING
//#define EP_CFG_LOGGING
//#define EP_UDP_LOGGING
//#define EP_COAP_LOGGING
#define EP_THREAD_LOGGING

#endif /* APP_CONFIG_H_ */

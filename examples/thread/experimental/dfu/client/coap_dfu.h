/* Copyright (c) 2016 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup thread_secure_dfu_example_client coap_dfu.h
 * @{
 * @ingroup thread_secure_dfu_example
 * @brief Thread Secure DFU Example - DFU client interface.
 *
 */

#ifndef COAP_DFU_H_
#define COAP_DFU_H_

#include "coap_api.h"

/**
 * @brief Initialize DFU client.
 */
uint32_t coap_dfu_init(void);

/**
 * @brief Trigger DFU.
 *
 * @param[in] p_remote Address of a host which should be queried for DFU. If NULL then the
 *                     function will attempt to discover any DFU hosts by sending
 *                     a trigger request to a multicast address.
 */
uint32_t coap_dfu_trigger(coap_remote_t * p_remote);

#endif /* COAP_DFU_H_ */

/** @} */

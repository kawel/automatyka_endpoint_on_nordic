/**
 * @file ex_light.h
 * @author Pawel Radecki
 * @date 26 Nov 2017
 * @brief File contains functionality which allows to run default example from multiprotocol.
 * This file is created to allow testing of the Thread + BLE on the
 * Evaluation Boards only (no PC + NCP or HUB is required).
 * How to use it check example BLE UART and Thread MTD CoAP Client Examples
 * which is based on the: ble_thread_dynamic_mtd_coap_client + simple_coap_server
 *
 */

#ifndef EX_LIGHT_H_
#define EX_LIGHT_H_

#include <openthread/openthread.h>

#define COMMAND_REQUEST_UNICAST         "u"
#define COMMAND_REQUEST_MULTICAST       "m"
#define COMMAND_REQUEST_PROVISIONING    "p"


#ifdef __cplusplus
extern "C" {
#endif

void ex_light_init(otInstance *ot_instance);
void ex_light_set_peer_address_unspecified(void);

void ex_light_unicast(void);
void ex_light_multicast(void);
void ex_light_provisioning(void);

void thread_command_handler(const uint8_t * p_command_str, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* EX_LIGHT_H_ */

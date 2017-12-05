/**
 * @file ep_thread.h
 * @author Pawel Radecki
 * @date 04 Dec 2017
 * @brief File defines interface to initialize Thread instance.
 *
 */

#ifndef EP_THREAD_H_
#define EP_THREAD_H_

#include <openthread/openthread.h>

#ifdef __cplusplus
extern "C" {
#endif

otInstance * ep_thread_init(void);

#ifdef __cplusplus
}
#endif

#endif /* EP_THREAD_H_ */

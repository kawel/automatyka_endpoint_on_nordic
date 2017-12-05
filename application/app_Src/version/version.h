/**
 * @file version.h
 * @author Pawel Radecki
 * @date 25 Sep 2017
 * @brief File defines interface to obtain software version related data.
 *
 */

#ifndef VERSION_H_
#define VERSION_H_

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Get program name
 *
 * If @p buflen is zero, no bytes are stored in @p buf.
 * In this case, is returned the number of bytes that would be the length of the
 * string. It does not matter what @p buf is if @p buflen is zero (may even be
 * a null pointer).
 *
 * @param[out] buf    A pointer to buffer
 * @param[in]  buflen Size of the buffer
 * @return Number of bytes that are the length of the string to return (excluding
 *         the trailing null)
 */
size_t VER_get_program_name(char *buf, size_t buflen);

/**
 * @brief  Get software revision
 *
 * If @p buflen is zero, no bytes are stored in @p buf.
 * In this case, is returned the number of bytes that would be the length of the
 * string. It does not matter what @p buf is if @p buflen is zero (may even be
 * a null pointer).
 *
 * @param[out] buf    A pointer to buffer
 * @param[in]  buflen Size of the buffer
 * @return Number of bytes that are the length of the string to return (excluding
 *         the trailing null)
 */
size_t VER_get_sw_revision(char *buf, size_t buflen);

/**
 * @brief  Get time of the compilation
 *
 * If @p buflen is zero, no bytes are stored in @p buf.
 * In this case, is returned the number of bytes that would be the length of the
 * string. It does not matter what @p buf is if @p buflen is zero (may even be
 * a null pointer).
 *
 * @param[out] buf    A pointer to buffer
 * @param[in]  buflen Size of the buffer
 * @return Number of bytes that are the length of the string to return (excluding
 *         the trailing null)
 */
size_t VER_get_compilation_time(char *buf, size_t buflen);

/**
 * @brief  Get date of the compilation
 *
 * If @p buflen is zero, no bytes are stored in @p buf.
 * In this case, is returned the number of bytes that would be the length of the
 * string. It does not matter what @p buf is if @p buflen is zero (may even be
 * a null pointer).
 *
 * @param[out] buf    A pointer to buffer
 * @param[in]  buflen Size of the buffer
 * @return Number of bytes that are the length of the string to return (excluding
 *         the trailing null)
 */
size_t VER_get_compilation_date(char *buf, size_t buflen);

#ifdef __cplusplus
}
#endif

#endif /* VERSION_H_ */

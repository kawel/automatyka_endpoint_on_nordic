/**
 * @file utils.h
 * @author Pawel Radecki
 * @date 11 Nov 2017
 * @brief This file includes macros for validating runtime conditions.
 *
 * CONTEN OF THE FILE COPIED FROM code_utils.hpp FROM NEWER VERSION OF THE LIB.
 */

#ifndef UTILS_H_
#define UTILS_H_

/**
 *  This checks for the specified status, which is expected to
 *  commonly be successful, and branches to the local label 'exit' if
 *  the status is unsuccessful.
 *
 *  @param[in]  aStatus     A scalar status to be evaluated against zero (0).
 *
 */
#define SuccessOrExit(aStatus)                      \
    do                                              \
    {                                               \
        if ((aStatus) != 0)                         \
        {                                           \
            goto exit;                              \
        }                                           \
    } while (false)

/**
 *  This checks for the specified condition, which is expected to
 *  commonly be true, and both executes @a ... and branches to the
 *  local label 'exit' if the condition is false.
 *
 *  @param[in]  aCondition  A Boolean expression to be evaluated.
 *  @param[in]  ...         An expression or block to execute when the
 *                          assertion fails.
 *
 */
#define VerifyOrExit(aCondition, ...)                 \
    do                                                \
    {                                                 \
        if (!(aCondition))                            \
        {                                             \
            __VA_ARGS__;                              \
            goto exit;                                \
        }                                             \
    } while (false)

/**
 *  This unconditionally executes @a ... and branches to the local
 *  label 'exit'.
 *
 *  @note The use of this interface implies neither success nor
 *        failure for the overall exit status of the enclosing
 *        function body.
 *
 *  @param[in]  ...         An optional expression or block to execute
 *                          when the assertion fails.
 *
 */
#define ExitNow(...)                            \
    do                                          \
    {                                           \
        __VA_ARGS__;                            \
        goto exit;                              \
    } while (false)

/*
 * Run the `statement` and ignore the return value.
 *
 * This is primarily used to indicate the intention of developer that
 * the return value of function/method can be safely ignored.
 *
 * @param[in]  aStatement  The function/method to execute.
 *
 */
#define IgnoreReturnValue(aStatement)          \
    do                                         \
    {                                          \
        if (aStatement) {}                     \
    } while (false)



#endif /* UTILS_H_ */

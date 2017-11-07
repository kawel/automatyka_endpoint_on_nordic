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
 * @defgroup thread_secure_dfu_example_client coap_dfu.c
 * @{
 * @ingroup thread_secure_dfu_example
 * @brief Thread Secure DFU Example - DFU client implementation.
 *
 */
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "sdk_config.h"
#include "boards.h"
#include "app_util.h"
#include "app_timer.h"
#include "nrf_error.h"
#include "nrf_dfu_settings.h"
#include "nrf_dfu_req_handler.h"
#include "nordic_common.h"
#include "coap_block.h"
#include "coap_option.h"
#include "openthread/platform/random.h"
#include "coap_dfu.h"

#define NRF_LOG_LEVEL 4
#define NRF_LOG_MODULE_NAME "COAP_DFU"
#include "nrf_log.h"

#define DFU_RESOURCE_PREFIX     "dfu"

/**
 * @brief Default block size for CoAP blocwise transfers. The default
 * value ensures that blocks aren't fragmented on 802.15.4
 */
#define DEFAULT_BLOCK_SIZE      64
/**
 * @brief Defines how many retries are performed in case
 * a NON CoAP request is timed out.
 */
#define DEFAULT_NON_RETRIES     COAP_MAX_RETRANSMIT_COUNT
/**
 * @brief Defines how many retries are performed in case
 * a CON CoAP request is timed out.
 */
#define DEFAULT_CON_RETRIES     3

/** @brief DFU trigger packet version. */
#define TRIGGER_VERSION         0

/** @brief Maximum delay (in ms) between requesting
 * consecutive image blocks.
 */
#define DEFAULT_DELAY_MAX_MS    128

#define INIT_RESOURCE_NAME      "init"
#define IMAGE_RESOURCE_NAME     "image"
#define TRIGGER_RESOURCE_NAME   "trig"
#define RESOURCE_PATH(name)     (DFU_RESOURCE_PREFIX "/" name)

#define REALM_LOCAL_ADDR        (uint8_t []){0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}

/** @brief DFU client state ID.
 *
 * We reuse DFU object type IDs as DFU process states IDs,
 * so that current state can be used as the object type in
 * function which expect one.
 */
typedef enum
{
	DFU_DOWNLOAD_INIT_CMD = NRF_DFU_OBJ_TYPE_COMMAND,
	DFU_DOWNLOAD_FIRMWARE = NRF_DFU_OBJ_TYPE_DATA,
    DFU_DOWNLOAD_TRIG,
    DFU_IDLE,
	DFU_ERROR,
} dfu_state_t;

/** @brief DFU event definitions. */
typedef enum
{
    DFU_EVENT_TRANSFER_COMPLETE,
    DFU_EVENT_TRANSFER_CONTINUE,
    DFU_EVENT_TRANSFER_ERROR,
    DFU_EVENT_PROCESSING_ERROR,
} dfu_even_t;

/** @brief Trigger packet structure. */
typedef PACKED_STRUCT {
    uint8_t flags;
    uint32_t init_length;
    uint32_t init_crc;
    uint32_t image_length;
    uint32_t image_crc;
} dfu_trigger_t;

struct dfu_context;

/** @brief Resource handler callback. */
typedef uint32_t (*response_handler_t)(struct dfu_context      * p_dfu_ctx,
                                       coap_message_t          * p_message,
                                       coap_block_opt_block2_t * p_block_opt);

static coap_message_t * create_request(coap_remote_t * p_remote,
                                       const char    * p_resource_path,
                                       uint16_t        block_size,
                                       uint32_t        block_num);

static void response_handler(uint32_t status, void * p_arg, coap_message_t * p_response);
static uint32_t dfu_handle_event(dfu_even_t event);

/** @brief DFU client state. */
typedef struct dfu_context
{
    coap_resource_t    root;             /**< Root resource. */
    coap_resource_t    prefix;           /**< Prefix resource. */
    coap_resource_t    trigger;          /**< Trigger resource. */

    dfu_state_t        dfu_state;        /**< Current DFU client state. */
    response_handler_t handler;          /**< A pointer to the current response handler. */

    uint32_t           init_cmd_size;    /**< Current init command size. */
    uint32_t           init_cmd_crc;     /**< Current init command checksum. */
    uint32_t           firmware_size;    /**< Current firmware command size. */
    uint32_t           firmware_crc;     /**< Current firmware command checksum. */
    uint32_t           max_obj_size;     /**< Maximum size of the DFU object. */

    uint32_t           remaining_size;   /**< Remaining size, in bytes, of the resource which
                                              is being downloaded. */
    uint32_t           block_num;        /**< Currently requested block number. */
    uint32_t      *    p_resource_size;  /**< Downloaded resource size. */
    const char    *    resource_path;    /**< Downloaded resource path on the remote host. */
    coap_remote_t      remote;           /**< Remote host from which the resource is being downloaded. */

    uint8_t            retry_count;      /**< Number of remaining retires. */
    bool               timer_active;     /**< True if a CoAP request is pending, false otherwise. */
} dfu_context_t;

/** @brief Coap Message Id. */
extern uint16_t      m_coap_message_id;

/** @brief CoAP token. */
static uint16_t      m_coap_token;

/** @brief DFU client context. */
static dfu_context_t m_dfu_ctx;

/** @brief Request delay timer. */
APP_TIMER_DEF(m_send_timer);

/** @brief Helper function convering DFU state to string.
 *  @param[in] state DFU client state.
 *  @return a pointer to null terminated string with state name.
 */
static const char * dfu_state_to_string(const dfu_state_t state)
{
    static const char * const names[] =
    {
        "DFU_DOWNLOAD_INIT_CMD",
        "DFU_DOWNLOAD_FIRMWARE",
        "DFU_DOWNLOAD_TRIG",
        "DFU_IDLE",
        "DFU_ERROR",
    };

    return names[state - DFU_DOWNLOAD_INIT_CMD];
}

/** @brief Helper function convering DFU event name to string.
 *  @param[in] state DFU client event.
 *  @return a pointer to null terminated string with event name.
 */
static const char * dfu_event_to_string(const dfu_even_t event)
{
    static const char * const names[] = {
        "DFU_EVENT_TRANSFER_COMPLETE",
        "DFU_EVENT_TRANSFER_CONTINUE",
        "DFU_EVENT_TRANSFER_ERROR",
        "DFU_EVENT_PROCESSING_ERROR",
    };

    return names[event];
}

/** @brief Select DFU object.
 *  @param[in]  object_type    Object type which should be selected.
 *  @param[out] p_max_obj_size Returns max size of an object of the selected type.
 *  @param[out] p_offset       Returns current offset (size) of an object of the selected type.
 *
 *  @return Operation result code.
 */
static uint32_t dfu_op_select(uint32_t object_type, uint32_t * p_max_obj_size, uint32_t * p_offset)
{
    nrf_dfu_res_code_t  res_code;
    nrf_dfu_req_t       dfu_req;
    nrf_dfu_res_t       dfu_res;

    memset(&dfu_req, 0, sizeof(dfu_req));

    dfu_req.req_type = NRF_DFU_OBJECT_OP_SELECT;
    dfu_req.obj_type = object_type;

    res_code = nrf_dfu_req_handler_on_req(NULL, &dfu_req, &dfu_res);
    if (res_code == NRF_DFU_RES_CODE_SUCCESS)
    {
        NRF_LOG_INFO("Object selected max_size: %u offs: %u crc: %x\r\n",
                     dfu_res.max_size, dfu_res.offset, dfu_res.crc);

        *p_max_obj_size = dfu_res.max_size;
        *p_offset       = dfu_res.offset;
    }

    return res_code;
}

/** @brief Create DFU object.
 *  @param[in] object_type  Object type which should be selected.
 *  @param[in] object_size  Size of an object to create.
 *
 *  @return Operation result code.
 */
static uint32_t dfu_op_create(uint32_t object_type, uint32_t object_size)
{
    nrf_dfu_res_code_t  res_code;
    nrf_dfu_req_t       dfu_req;
    nrf_dfu_res_t       dfu_res;

    memset(&dfu_req, 0, sizeof(dfu_req));

    dfu_req.req_type    = NRF_DFU_OBJECT_OP_CREATE;
    dfu_req.object_size = object_size;
    dfu_req.obj_type    = object_type;

    res_code = nrf_dfu_req_handler_on_req(NULL, &dfu_req, &dfu_res);
    if (res_code == NRF_DFU_RES_CODE_SUCCESS)
    {
        NRF_LOG_INFO("Object created [type=%u, size=%u]\r\n", object_type, object_size);
    }

    return res_code;
}

/**
 *  @brief Write DFU object.
 *  @param[in] object_type     Object type which should be selected.
 *  @param[in] p_payload       A pointer to data which should be written to the object.
 *  @param[in] payload_length  Length, in bytes, of data which should be written to the object.
 *
 *  @return Operation result code.
 */
static uint32_t dfu_op_write(uint32_t        object_type,
                             const uint8_t * p_payload,
                             uint16_t        payload_length)
{
    nrf_dfu_res_code_t  res_code;
    nrf_dfu_req_t       dfu_req;
    nrf_dfu_res_t       dfu_res;

    memset(&dfu_req, 0, sizeof(dfu_req));

    dfu_req.obj_type = object_type;
    dfu_req.req_type = NRF_DFU_OBJECT_OP_WRITE;
    dfu_req.p_req    = (uint8_t *)p_payload;
    dfu_req.req_len  = payload_length;

    res_code = nrf_dfu_req_handler_on_req(NULL, &dfu_req, &dfu_res);
    if (res_code == NRF_DFU_RES_CODE_SUCCESS)
    {
        NRF_LOG_INFO("Object written: %u \r\n", payload_length);
    }

    return res_code;
}

/**
 *  @brief Calculate DFU object CRC.
 *
 *  @param[in] object_type Object type which should be selected.
 *
 *  @return Operation result code.
 */
static uint32_t dfu_op_crc(uint32_t object_type)
{
    nrf_dfu_res_code_t  res_code;
    nrf_dfu_req_t       dfu_req;
    nrf_dfu_res_t       dfu_res;

    memset(&dfu_req, 0, sizeof(dfu_req));

    // Calculate CRC of the object
    dfu_req.obj_type = object_type;
    dfu_req.req_type = NRF_DFU_OBJECT_OP_CRC;

    res_code = nrf_dfu_req_handler_on_req(NULL, &dfu_req, &dfu_res);
    if (res_code == NRF_DFU_RES_CODE_SUCCESS)
    {
        NRF_LOG_INFO("Object CRC: %x\r\n", dfu_res.crc);
    }

    return res_code;
}

/**
 *  @brief Execute selected DFU.
 *
 *  @param[in] object_type Object type which should be selected.
 *
 *  @return Operation result code.
 */
static uint32_t dfu_op_execute(uint32_t object_type)
{
    nrf_dfu_res_code_t  res_code;
    nrf_dfu_req_t       dfu_req;
    nrf_dfu_res_t       dfu_res;

    memset(&dfu_req, 0, sizeof(dfu_req));

    dfu_req.obj_type = object_type;
    dfu_req.req_type = NRF_DFU_OBJECT_OP_EXECUTE;

    res_code = nrf_dfu_req_handler_on_req(NULL, &dfu_req, &dfu_res);
    if (res_code == NRF_DFU_RES_CODE_SUCCESS)
    {
        NRF_LOG_INFO("Object executed\r\n");
    }

    return res_code;
}

/**
 * @brief Check if installed image is diffetrent from the incoming one.
 *
 * @param[in] p_dfu_ctx a pointer to DFU client context.
 *
 * @return True if image different, false otherwise.
 *
 */
static bool is_image_different(const dfu_context_t * p_dfu_ctx)
{
    if (s_dfu_settings.bank_0.bank_code == NRF_DFU_BANK_INVALID)
    {
        NRF_LOG_DEBUG("No image in bank 0");
        return true;
    }

    if (s_dfu_settings.bank_0.image_crc != p_dfu_ctx->firmware_crc)
    {
        NRF_LOG_DEBUG("Installed image CRC is different\r\n");
        return true;
    }

    return false;
}

/**
 * @brief Check if stored init command is valid.
 *
 * @param[in]  p_dfu_ctx a pointer to DFU client context.
 * @param[out] p_offset  an offset of completed data if init command is incomplete.
 *
 * @return True if init command is valid, false otherwise.
 *
 */
static bool is_init_command_valid(dfu_context_t * p_dfu_ctx, uint32_t * p_offset)
{
    uint32_t err_code;

    err_code = dfu_op_select(NRF_DFU_OBJ_TYPE_COMMAND,
                             &p_dfu_ctx->max_obj_size,
                             p_offset);

    if (err_code != NRF_DFU_RES_CODE_SUCCESS)
    {
        NRF_LOG_DEBUG("Select failed\r\n");
        return false;
    }

    // Check if a valid init command is present.
    err_code = dfu_op_execute(NRF_DFU_OBJ_TYPE_COMMAND);
    if (err_code != NRF_DFU_RES_CODE_SUCCESS)
    {
        NRF_LOG_ERROR("No valid init command\r\n");
        return false;
    }

    // Check if init command is the same as before.
    if (s_dfu_settings.progress.command_crc != p_dfu_ctx->init_cmd_crc)
    {
        NRF_LOG_ERROR("Init commad has changed\r\n");
        *p_offset = 0;
        p_dfu_ctx->remaining_size = 0;
        return false;
    }

    return true;
}

/** @brief Check if given address is real local.
 *
 *  @param[in] a pointer to the address.
 *
 *  @return True if address is realm local, false otherwise.
 **/
static bool is_addr_realm_local(const coap_remote_t * p_remote)
{
    return ((p_remote->addr[0] == 0xff) && (p_remote->addr[1] == 0x03));
}

/** @brief Check if given address is link local.
 *
 *  @param[in] a pointer to the address.
 *
 *  @return True if address is link local, false otherwise.
 **/
static bool is_addr_link_local(const coap_remote_t * p_remote)
{
    return ((p_remote->addr[0] == 0xff) && (p_remote->addr[1] == 0x02));
}

/** @brief Parse and return CoAP block option from a given message.
 *
 *  @param[in]  p_message        a pointer to CoAP message.
 *  @param[out] p_block2_option  a pointer to parsed block 2 option.
 *
 *  @return True if message has Block2 option, false otherwise.
 */
static bool get_block2_opt(coap_message_t          * p_message,
                           coap_block_opt_block2_t * p_block2_option)
{
    uint32_t        err_code;
    uint8_t         option_index;
    uint32_t        option_value;
    coap_option_t * p_option;

    if (coap_message_opt_index_get(&option_index, p_message, COAP_OPT_BLOCK2) != NRF_SUCCESS)
    {
        return false;
    }

    p_option = &p_message->options[option_index];
    err_code = coap_opt_uint_decode(&option_value,
                                    p_option->length,
                                    p_option->p_data);

    if (err_code != NRF_SUCCESS)
    {
        return false;
    }

    err_code = coap_block_opt_block2_decode(p_block2_option, option_value);

    return (err_code == NRF_SUCCESS);
}

/** @brief Set CoAP block2 option on a given message.
 *
 *  @param[inout]  p_message    a pointer to CoAP message.
 *  @param[in]     block_size   block size to set
 *  @param[in]     block_number block number to set
 *
 *  @return NRF_SUCCESS if succesful, error code otherwise.
 */
static uint32_t set_block2_opt(coap_message_t * p_message,
                               uint16_t         block_size,
                               uint32_t         block_number)
{
    uint32_t err_code;
    uint32_t option_value;

    coap_block_opt_block2_t block2_option =
    {
        .number = block_number,
        .size   = block_size
    };

    err_code = coap_block_opt_block2_encode(&option_value, &block2_option);
    if (err_code == NRF_SUCCESS)
    {
        err_code = coap_message_opt_uint_add(p_message,
                                             COAP_OPT_BLOCK2,
                                             option_value);
    }

    return err_code;
}

/** @brief Set CoAP URI option on a given message.
 *
 *  @param[inout]  p_message   a pointer to CoAP message.
 *  @param[in]     p_resource  an URI as NULL terminated string which should be
 *                             converted to CoAP options.
 *
 *  @return NRF_SUCCESS if succesful, error code otherwise.
 */
static uint32_t set_uri_opt(coap_message_t * p_message,
                            const char     * p_resource)
{
    uint32_t     err_code;
    const char * p_start = p_resource;
    const char * p_end;
    uint32_t     len;

    if (*p_start == '/')
    {
        p_start++;
    }

    do {
        p_end = strchr(p_start, '/');
        if (p_end)
        {
            len = p_end - p_start;
        }
        else
        {
            len = strlen(p_start);
        }

        err_code = coap_message_opt_str_add(p_message,
                                            COAP_OPT_URI_PATH,
                                            (uint8_t *) p_start,
                                            len);
        p_start += (len + 1);

    } while ((err_code == NRF_SUCCESS) && p_end);

    return err_code;
}

/**
 * @brief Parses trigger data and updates DFU client context accordingly.
 *
 * @param[inout] p_dfu_ctx a pointer to DFU Client context.
 * @param[in]    p_trigger a pointer to trigger data.
 *
 * @return True if parsing was successful, false otherwise.
 */
static bool parse_trigger(dfu_context_t * p_dfu_ctx, const dfu_trigger_t * p_trigger)
{
    if (((p_trigger->flags >> 4) & 0x03) == TRIGGER_VERSION)
    {
#if defined ( __ICCARM__ )
    // Unaligned member access is not a problem in this particular case.
    #pragma diag_suppress=Pa039
#endif
        p_dfu_ctx->init_cmd_size = uint32_big_decode((const uint8_t *)&p_trigger->init_length);
        p_dfu_ctx->init_cmd_crc  = uint32_big_decode((const uint8_t *)&p_trigger->init_crc);
        p_dfu_ctx->firmware_size = uint32_big_decode((const uint8_t *)&p_trigger->image_length);
        p_dfu_ctx->firmware_crc  = uint32_big_decode((const uint8_t *)&p_trigger->image_crc);

        NRF_LOG_INFO("DFU trigger: init (sz=%d, crc=%0X) image (sz=%d, crc=%0X)\r\n",
                     p_dfu_ctx->init_cmd_size,
                     p_dfu_ctx->init_cmd_crc,
                     p_dfu_ctx->firmware_size,
                     p_dfu_ctx->firmware_crc);

        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief Check if resource prefix in the given message is equal to @ref DFU_RESOURCE_PREFIX.
 *
 * @param[in] p_dfu_ctx a pointer to DFU client context.
 * @param[in] p_message a pointer to message.
 *
 * @return True if prefix is equal, false otherwise.
 */
static bool is_prefix_equal(dfu_context_t  * p_dfu_ctx,
                            coap_message_t * p_message)
{
    uint8_t path_segment = 0;

    for (uint32_t index = 0; index < p_message->options_count; index++)
    {
        if (p_message->options[index].number == COAP_OPT_URI_PATH)
        {
            if (path_segment != 0)
            {
                return false;
            }

            if (p_message->options[index].length != strlen(DFU_RESOURCE_PREFIX))
            {
                return false;
            }

            if (memcmp(p_message->options[index].p_data,
                       DFU_RESOURCE_PREFIX,
                       p_message->options[index].length) != 0)
            {
                return false;
            }

            return true;
        }
    }

    return false;
}

/**
 * @brief Trigger request response handler.
 *
 * The function is called when a response to eariler trigger request has been received.
 *
 * @param[in] p_dfu_ctx   a pointer to DFU client context.
 * @param[in] p_message   a pointer to message which contains the response.
 * @param[in] p_block_opt a pointer to block options.
 *
 * @return Operation result code.
 */
static uint32_t trigger_handle_response(dfu_context_t           * p_dfu_ctx,
                                        coap_message_t          * p_message,
                                        coap_block_opt_block2_t * p_block_opt)
{
    do
    {
        if (memcmp(p_message->token, &m_coap_token, 2) != 0)
        {
            NRF_LOG_WARNING("Token mismatch\r\n");
            break;
        }

        if (p_message->payload_len != sizeof(dfu_trigger_t))
        {
            NRF_LOG_INFO("Trigger size mismatch\r\n");
            break;
        }

        if (!parse_trigger(p_dfu_ctx, (const dfu_trigger_t *)p_message->p_payload))
        {
            NRF_LOG_ERROR("Trigger parsing failed\r\n");
            break;
        }

        // If the initial /trig request was sent to a multicast address, we update
        // the address to the address of the DFU server which replied first.
        if (is_addr_link_local(&p_dfu_ctx->remote) || is_addr_realm_local(&p_dfu_ctx->remote))
        {
            NRF_LOG_INFO("Remote address updated\r\n");
            memcpy(&p_dfu_ctx->remote, &p_message->remote, sizeof(p_dfu_ctx->remote));
        }

        dfu_handle_event(DFU_EVENT_TRANSFER_COMPLETE);

    } while (0);

    return NRF_SUCCESS;
}

/**
 * @brief Handle reponse payload.
 *
 * @param[in] p_context   a ponter to DFU client context.
 * @param[in] p_message   a pointer to the message which contains the response payload.
 * @param[in] p_block_opt a pointer to block options.
 *
 * @return Operation result code.
 */
static uint32_t handle_payload(dfu_context_t           * p_context,
                               coap_message_t          * p_message,
                               coap_block_opt_block2_t * p_block_opt)
{
    uint32_t obj_type = m_dfu_ctx.dfu_state;
    uint32_t res_code = NRF_DFU_RES_CODE_SUCCESS;

    if (memcmp(p_message->token, &m_coap_token, 2) != 0)
    {
        NRF_LOG_WARNING("Token mismatch\r\n");
        return res_code;
    }

    if (p_context->remaining_size == 0)
    {
        uint32_t offset = p_block_opt->number*p_block_opt->size;
        p_context->remaining_size = MIN(m_dfu_ctx.max_obj_size, (*m_dfu_ctx.p_resource_size - offset));
        res_code = dfu_op_create(obj_type, p_context->remaining_size);
        NRF_LOG_INFO("Transfer offset: %u\r\n", offset);
    }

    if (res_code != NRF_DFU_RES_CODE_SUCCESS)
    {
        dfu_handle_event(DFU_EVENT_PROCESSING_ERROR);
        return res_code;
    }

    res_code = dfu_op_write(obj_type, p_message->p_payload, p_message->payload_len);
    if (res_code != NRF_DFU_RES_CODE_SUCCESS)
    {
        dfu_handle_event(DFU_EVENT_PROCESSING_ERROR);
        return res_code;
    }

    p_context->remaining_size -= p_message->payload_len;

    if (p_context->remaining_size == 0)
    {
        res_code = dfu_op_crc(obj_type);
        if (res_code != NRF_DFU_RES_CODE_SUCCESS)
        {
            dfu_handle_event(DFU_EVENT_PROCESSING_ERROR);
            return res_code;
        }

        res_code = dfu_op_execute(obj_type);
        if (res_code != NRF_DFU_RES_CODE_SUCCESS)
        {
            dfu_handle_event(DFU_EVENT_PROCESSING_ERROR);
            return res_code;
        }
    }

    if (p_block_opt->more)
    {
        m_dfu_ctx.block_num++;
        dfu_handle_event(DFU_EVENT_TRANSFER_CONTINUE);
    }
    else
    {
        nrf_dfu_req_handler_reset_if_dfu_complete();
        dfu_handle_event(DFU_EVENT_TRANSFER_COMPLETE);
    }

    return res_code;
}

/** @brief Handler called upon receiving a respone to COAP GET request.
 */
static void response_handler(uint32_t status, void * p_arg, coap_message_t * p_response)
{
    NRF_LOG_DEBUG("Request callback for resource [%s]\r\n", (uint32_t)m_dfu_ctx.resource_path);

    if (status == NRF_SUCCESS)
    {
        if (p_response->header.code == COAP_CODE_205_CONTENT)
        {
            coap_block_opt_block2_t block_opt = {0};
            bool has_block = get_block2_opt(p_response, &block_opt);
            if (has_block)
            {
                NRF_LOG_DEBUG("Received block %3lu\r\n", block_opt.number);
            }

            // Check if received block number is the same as requested one. This
            // is a guard against misbehaving COAP servers.
            if (block_opt.number == m_dfu_ctx.block_num)
            {
                m_dfu_ctx.handler(&m_dfu_ctx, p_response, &block_opt);
            }
            else
            {
                NRF_LOG_WARNING("Requested %d but got %d\r\n", m_dfu_ctx.block_num, block_opt.number);
                dfu_handle_event(DFU_EVENT_TRANSFER_CONTINUE);
            }
        }
        else
        {
            NRF_LOG_WARNING("Request response code: %d\r\n", p_response->header.code);
            dfu_handle_event(DFU_EVENT_TRANSFER_ERROR);
        }

    }
    else if (status == COAP_TRANSMISSION_TIMEOUT)
    {
        NRF_LOG_WARNING("Request timeout\r\n");
        dfu_handle_event(DFU_EVENT_TRANSFER_ERROR);
    }
}

/** @brief Timer callback which sends eariler scheduled request.
 *
 *  @param[in] context A pointer to coap message which should be sent.
 */
static void send_message(void * p_context)
{
    coap_message_t * p_request = (coap_message_t *)p_context;
    uint32_t handle;


    NRF_LOG_INFO("Sending message [mid:%d]\r\n", p_request->header.id);
    if (coap_message_send(&handle, p_request) != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Failed to send CoAP message\r\n");
    }

    m_dfu_ctx.timer_active = false;
    coap_message_delete(p_request);

    LEDS_INVERT(BSP_LED_2_MASK);
}

/**
 * @brief Create COAP GET request.
 *
 * @param[in] p_remote         A pointer to host.
 * @param[in] p_resource_path  An URI of the resource which should be requested.
 * @param[in] block_size       Requested block size.
 * @param[in] block_num        Requested block number.
 *
 * @return A pointer to CoAP message or NULL on error.
 */
static coap_message_t * create_request(coap_remote_t * p_remote,
                                       const char    * p_resource_path,
                                       uint16_t        block_size,
                                       uint32_t        block_num)
{
	uint32_t              err_code;
	coap_message_t      * p_request;
	coap_message_conf_t   message_conf;

	if (is_addr_realm_local(p_remote) || is_addr_link_local(p_remote))
	{
        message_conf.type = COAP_TYPE_NON;
	}
	else
	{
	    message_conf.type = COAP_TYPE_CON;
	}
	message_conf.code              = COAP_CODE_GET;
	message_conf.port.port_number  = DFU_UDP_PORT;
	message_conf.id                = m_coap_message_id;
	message_conf.response_callback = response_handler;
	message_conf.token_len         = 2;

	m_coap_token = (uint16_t)otPlatRandomGet();
	memcpy(message_conf.token, &m_coap_token, 2);

	m_coap_message_id++;

	err_code = coap_message_new(&p_request, &message_conf);
	if (err_code != NRF_SUCCESS)
	{
	    return NULL;
	}

	p_request->p_arg = (char *)p_resource_path;

	err_code = coap_message_remote_addr_set(p_request, p_remote);
    if (err_code != NRF_SUCCESS)
    {
        coap_message_delete(p_request);
        return NULL;
    }

    err_code = set_uri_opt(p_request, p_resource_path);
    if (err_code != NRF_SUCCESS)
    {
        coap_message_delete(p_request);
        return NULL;;
    }

    if (block_size > 0)
    {
        set_block2_opt(p_request, block_size, block_num);
    }

	return p_request;
}

/**
 * @brief Reset state machine state.
 *
 * @param[in] p_dfu_ctx a pointer to DFU client context.
 */
static void dfu_reset_state(dfu_context_t * p_dfu_ctx)
{
    p_dfu_ctx->dfu_state     = DFU_IDLE;
    p_dfu_ctx->init_cmd_size = 0;
    p_dfu_ctx->firmware_size = 0;
    p_dfu_ctx->remaining_size      = 0;
}

/**
 * @brief DFU state machine handler.
 *
 * @param[in] event DFU event
 * @return NRF_SUCCESS or error code
 */
static uint32_t dfu_handle_event(dfu_even_t event)
{
    uint32_t err_code = NRF_SUCCESS;

    NRF_LOG_INFO("state=%s event=%s\r\n",
                  (uint32_t)dfu_state_to_string(m_dfu_ctx.dfu_state),
                  (uint32_t)dfu_event_to_string(event));

    switch (m_dfu_ctx.dfu_state)
    {
        case DFU_IDLE:
        {
            if (event == DFU_EVENT_TRANSFER_COMPLETE)
            {
                m_dfu_ctx.dfu_state     = DFU_DOWNLOAD_TRIG;
                m_dfu_ctx.resource_path = RESOURCE_PATH(TRIGGER_RESOURCE_NAME);
                m_dfu_ctx.block_num     = 0;
                m_dfu_ctx.handler       = trigger_handle_response;
                m_dfu_ctx.retry_count   = DEFAULT_NON_RETRIES;
            }
            break;
        }

        case DFU_DOWNLOAD_TRIG:
        {
            if (event == DFU_EVENT_TRANSFER_COMPLETE)
            {
                if (!is_image_different(&m_dfu_ctx))
                {
                    NRF_LOG_INFO("Image is already installed\r\n");
                    dfu_reset_state(&m_dfu_ctx);
                    break;
                }

                m_dfu_ctx.dfu_state = DFU_DOWNLOAD_INIT_CMD;
                uint32_t offset = 0;

                if (!is_init_command_valid(&m_dfu_ctx, &offset))
                {
                    m_dfu_ctx.resource_path   = RESOURCE_PATH(INIT_RESOURCE_NAME);
                    m_dfu_ctx.p_resource_size = &m_dfu_ctx.init_cmd_size;
                    m_dfu_ctx.handler         = handle_payload;
                    m_dfu_ctx.block_num       = (offset / DEFAULT_BLOCK_SIZE);
                    m_dfu_ctx.retry_count     = DEFAULT_CON_RETRIES;
                    break;
                }

                NRF_LOG_INFO("Valid init command found. Continue with firmware download.\r\n");
                // fallthrough - init command is completed and we should
                //               continue with firmware download
            }
        }

        /* no break */
        case DFU_DOWNLOAD_INIT_CMD:
        {
            if (event == DFU_EVENT_TRANSFER_COMPLETE)
            {
                uint32_t offset = 0;
                err_code = dfu_op_select(NRF_DFU_OBJ_TYPE_DATA,
                                         &m_dfu_ctx.max_obj_size,
                                         &offset);

                if (err_code != NRF_DFU_RES_CODE_SUCCESS)
                {
                    NRF_LOG_ERROR("Select failed\r\n");
                    m_dfu_ctx.dfu_state = DFU_ERROR;
                    break;
                }

                m_dfu_ctx.dfu_state       = DFU_DOWNLOAD_FIRMWARE;
                m_dfu_ctx.resource_path   = RESOURCE_PATH(IMAGE_RESOURCE_NAME);
                m_dfu_ctx.p_resource_size = &m_dfu_ctx.firmware_size;
                m_dfu_ctx.handler         = handle_payload;
                m_dfu_ctx.block_num       = (offset / DEFAULT_BLOCK_SIZE);
                m_dfu_ctx.retry_count     = DEFAULT_CON_RETRIES;
            }
            else if (event == DFU_EVENT_PROCESSING_ERROR)
            {
                //TODO: what now?
                m_dfu_ctx.dfu_state = DFU_ERROR;
                NRF_LOG_ERROR("Processing error while downloading init command.\r\n");
            }
            break;
        }

        case DFU_DOWNLOAD_FIRMWARE:
        {
            if (event == DFU_EVENT_TRANSFER_COMPLETE)
            {
                dfu_reset_state(&m_dfu_ctx);
            }
            else if (event == DFU_EVENT_PROCESSING_ERROR)
            {
                //TODO: what now?
                m_dfu_ctx.dfu_state = DFU_ERROR;
                NRF_LOG_ERROR("Processing error while downloading firmware.\r\n");
            }
            break;
        }

        default:
            NRF_LOG_ERROR("Unhandled state\r\n");
            break;
    }

    if (m_dfu_ctx.dfu_state != DFU_IDLE && m_dfu_ctx.dfu_state != DFU_ERROR)
    {
        if ((event == DFU_EVENT_TRANSFER_ERROR) && (m_dfu_ctx.retry_count > 0))
        {
            m_dfu_ctx.retry_count -= 1;
        }

        if (m_dfu_ctx.retry_count > 0)
        {
            if (!m_dfu_ctx.timer_active)
            {
                coap_message_t * p_request = create_request(&m_dfu_ctx.remote,
                                                            m_dfu_ctx.resource_path,
                                                            DEFAULT_BLOCK_SIZE,
                                                            m_dfu_ctx.block_num);

                uint8_t delay;
                do
                {
                    delay = (otPlatRandomGet() & (DEFAULT_DELAY_MAX_MS - 1));
                } while (delay == 0);

                app_timer_start(m_send_timer, APP_TIMER_TICKS(delay), p_request);
                m_dfu_ctx.timer_active = true;

                NRF_LOG_INFO("Requesting [%s] (block:%u mid:%d dly:%d)\r\n",
                             (uint32_t)m_dfu_ctx.resource_path,
                             m_dfu_ctx.block_num,
                             p_request->header.id,
                             delay);
            }
        }
        else
        {
            NRF_LOG_ERROR("No more retries\r\n");
            dfu_reset_state(&m_dfu_ctx);
        }
    }

    return err_code;
}

/**
 * @biref Trigger DFU
 *
 * @param[in] p_remote              Host address. If NULL then uses FF03::1
 * @param[in] skip_trigger_request  If true then skip trigger and proceed to init packet download.
 *
 * @return NRF_SUCCESS or error code
 */
static uint32_t trigger(const coap_remote_t * p_remote, bool skip_trigger_request)
{
    NRF_LOG_INFO("Triggering DFU\r\n");

    if (p_remote != NULL)
    {
        memcpy(&m_dfu_ctx.remote, p_remote, sizeof(*p_remote));
    }
    else
    {
        memcpy(&m_dfu_ctx.remote.addr, REALM_LOCAL_ADDR, 16);
        m_dfu_ctx.remote.port_number = DFU_UDP_PORT;
    }

    if (skip_trigger_request)
    {
        m_dfu_ctx.dfu_state = DFU_DOWNLOAD_TRIG;
    }

    return dfu_handle_event(DFU_EVENT_TRANSFER_COMPLETE);
}

/**
 * @brief This method is called when a request to trigger resource is received.
 *
 * @param[in] p_resource a pointer to resource
 * @param[in] p_request  a pointer to request
 */
static void trigger_callback(coap_resource_t * p_resource, coap_message_t * p_request)
{
    NRF_LOG_INFO("DFU trigger request received\r\n");

    do
    {
        if (p_request->header.code != COAP_CODE_POST)
        {
            NRF_LOG_DEBUG("Method not supported\r\n");
            break;
        }

        if (!is_prefix_equal(&m_dfu_ctx, p_request))
        {
            NRF_LOG_DEBUG("Prefix mismatch\r\n");
            break;
        }

        if (p_request->payload_len != sizeof(dfu_trigger_t))
        {
            NRF_LOG_DEBUG("Trigger size mismatch\r\n");
            break;
        }

        if (!parse_trigger(&m_dfu_ctx, (dfu_trigger_t *)p_request->p_payload))
        {
            NRF_LOG_DEBUG("Failed to parse trigger payload\r\n");
            break;
        }

        trigger(&p_request->remote, true);
    } while (0);
}

/**
 * @brief Initialize DFU Server resources.
 *
 * @param[in] p_dfu_ctx a pointer to DFU client context.
 */
static void endpoints_init(dfu_context_t * p_dfu_ctx)
{
    uint32_t err_code;

    do
    {
        err_code = coap_resource_create(&(p_dfu_ctx->root), "/");
        if (err_code != NRF_SUCCESS)
        {
            break;
        }

        err_code = coap_resource_create(&(p_dfu_ctx->prefix), DFU_RESOURCE_PREFIX);
        if (err_code != NRF_SUCCESS)
        {
            break;
        }

        err_code = coap_resource_create(&(p_dfu_ctx->trigger), TRIGGER_RESOURCE_NAME);
        if (err_code != NRF_SUCCESS)
        {
            break;
        }

        // We allow for GET method, so that the CoAP library doesn't send 'Method not allowed'
        // response. Methods other that POST are rejected in the callback.
        p_dfu_ctx->trigger.permission = (COAP_PERM_POST | COAP_PERM_GET);
        p_dfu_ctx->trigger.callback   = trigger_callback;

        err_code = coap_resource_child_add(&(p_dfu_ctx->prefix), &(p_dfu_ctx->trigger));
        if (err_code != NRF_SUCCESS)
        {
            break;
        }

        err_code = coap_resource_child_add(&(p_dfu_ctx->root), &(p_dfu_ctx->prefix));
        if (err_code != NRF_SUCCESS)
        {
            break;
        }

        NRF_LOG_INFO("Endpoints initialized\r\n");
    } while (0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// PUBLIC API
////////////////////////////////////////////////////////////////////////////////////////////////////


uint32_t coap_dfu_trigger(coap_remote_t * p_remote)
{
    if (m_dfu_ctx.dfu_state == DFU_IDLE)
    {
        return trigger(p_remote, false);
    }
    else
    {
        NRF_LOG_DEBUG("Invalid state\r\n");
        return NRF_ERROR_INVALID_STATE;
    }
}

uint32_t coap_dfu_init(void)
{
	uint32_t err_code = NRF_SUCCESS;

	endpoints_init(&m_dfu_ctx);

	nrf_dfu_settings_init();
    nrf_dfu_req_handler_init();

    dfu_reset_state(&m_dfu_ctx);

    app_timer_create(&m_send_timer, APP_TIMER_MODE_SINGLE_SHOT, send_message);

    return err_code;
}

/** @} */

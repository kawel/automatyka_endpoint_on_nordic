/**
 * @file ep_cfg.c
 * @author Pawel Radecki
 * @date 10 Nov 2017
 * @brief Configuration consist of data needed to attach to Thread network and
 *        in the future for BLE.
 *        Details about how configuration is passed and used is hidden in this
 *        implementation file.
 */


#include "ep_cfg.h"
#include "utils.h"

#include "nrf_log.h"
#include "boards.h"

#include "nfc_t4t_lib.h"
#include "nfc_ndef_msg.h"
#include "nfc_ndef_record_parser.h"


#ifdef EP_CFG_LOGGING
    #define EP_CFG_LOG_INFO(...)   NRF_LOG_INFO("EP CFG: " __VA_ARGS__)
    #define EP_CFG_LOG_ERROR(...)  NRF_LOG_ERROR("EP CFG: " __VA_ARGS__)
#else
    #define EP_CFG_LOG_INFO(...)
    #define EP_CFG_LOG_ERROR(...)
#endif


#define KIWI_NDEF_FILE_SIZE           128
#define TAG_TYPE_4_NLEN_FIELD_SIZE    2

typedef struct {
    uint16_t pan_id;
    uint8_t ext_pan_id[8];
    uint8_t master_key[16];
    uint8_t channel_id;
    uint16_t crc;
} __PACKED ep_cfg_t;


static ep_cfg_t ep_cfg;

uint8_t ndef_msg_buf[KIWI_NDEF_FILE_SIZE];
volatile bool ndef_msb_buf_updated;

static const uint8_t ndef_rec_field_type__kiwi[] =
    {'i', 'n', 'n', 'o', 'm', 'e', '.', 'c', 'o', 'm', ':','k', 'i', 'w', 'i'};


static unsigned short crc16(char *data_p, unsigned short length);
static __INLINE uint16_t REV16(uint16_t x);
static void ndef_process_data(otInstance *ot_instance);
static int ndef_validate_kiwi_data(nfc_ndef_bin_payload_desc_t *bin_pay_desc,
                                   nfc_ndef_record_desc_t *rec_desc,
                                   nfc_ndef_record_location_t *record_location);


static void nfc_callback(void          * context,
                         nfc_t4t_event_t event,
                         const uint8_t * data,
                         size_t          dataLength,
                         uint32_t        flags)
{
    (void)context;

    switch (event)
    {
        case NFC_T4T_EVENT_FIELD_ON:
            //EP_CFG_LOG_INFO("NFC T4T FIELD ON\r\n");
            break;

        case NFC_T4T_EVENT_FIELD_OFF:
            //EP_CFG_LOG_INFO("NFC T4T FIELD OFF\r\n");
            break;

        case NFC_T4T_EVENT_NDEF_READ:
            break;

        case NFC_T4T_EVENT_NDEF_UPDATED:
            if (dataLength != 0)
            {
                EP_CFG_LOG_INFO("NFC T4T NDEF UPDATED\r\n");
                ndef_msb_buf_updated = true;
            }
            break;

        default:
            break;
    }
}

static void nfc_init(void)
{
    EP_CFG_LOG_INFO("NFC Type 4 Tag Initialization\r\n");

    /* Set up NFC */
    SuccessOrExit(nfc_t4t_setup(nfc_callback, NULL));
    /* Run Read-Write mode for Type 4 Tag platform */
    SuccessOrExit(nfc_t4t_ndef_rwpayload_set(ndef_msg_buf, sizeof(ndef_msg_buf)));
    /* Start sensing NFC field */
    SuccessOrExit(nfc_t4t_emulation_start());

    EP_CFG_LOG_INFO("NFC T4T Emulation: Started.\r\n");

exit:
    return;
}

int ep_cfg_init(void)
{
    nfc_init();

    return 0;
}

int ep_cfg_check_and_apply(otInstance *ot_instance)
{
    if (true == ndef_msb_buf_updated)
    {
        EP_CFG_LOG_INFO("NDEF message buffer updated\r\n");

        ndef_msb_buf_updated = false;
        ndef_process_data(ot_instance);
    }

    return 0;
}

static void ndef_process_data(otInstance *ot_instance)
{
        nfc_ndef_bin_payload_desc_t bin_pay_desc;
        nfc_ndef_record_desc_t record_desc;
        nfc_ndef_record_location_t  record_location;
        uint32_t len = sizeof(ndef_msg_buf);

        if (NRF_SUCCESS == ndef_record_parser(&bin_pay_desc,
                                              &record_desc,
                                              &record_location,
                                              ndef_msg_buf + TAG_TYPE_4_NLEN_FIELD_SIZE,
                                              &len))
        {
            EP_CFG_LOG_INFO("NDEF record: Correct\r\n");
            if (0 == ndef_validate_kiwi_data(&bin_pay_desc, &record_desc, &record_location))
            {
                EP_CFG_LOG_INFO("NDEF record: Valid KIWI data\r\n");

                otThreadSetEnabled(ot_instance, false);
                EP_CFG_LOG_INFO("Open Thread: DISABLED.\r\n");

                // OpenThread: Configure
                SuccessOrExit(otThreadSetExtendedPanId(ot_instance, ep_cfg.ext_pan_id));
                SuccessOrExit(otLinkSetPanId(ot_instance, REV16(ep_cfg.pan_id)));
                EP_CFG_LOG_INFO("Open Thread: ExtPanId, PanId set.\r\n");

                otMasterKey masterKey;
                memcpy(masterKey.m8, ep_cfg.master_key, sizeof(ep_cfg.master_key));
                SuccessOrExit(otThreadSetMasterKey(ot_instance, &masterKey));
                EP_CFG_LOG_INFO("Open Thread: MasterKey set.\r\n");

                VerifyOrExit((ep_cfg.channel_id >= 11) && (ep_cfg.channel_id <= 26));
                SuccessOrExit(otLinkSetChannel(ot_instance, ep_cfg.channel_id));
                EP_CFG_LOG_INFO("Open Thread: Channel set.\r\n");

                // OpenThread: Start
                SuccessOrExit(otThreadSetEnabled(ot_instance, true));
                EP_CFG_LOG_INFO("Open Thread: ENABLED.\r\n");

                EP_CFG_LOG_INFO("NDEF record: KIWI configuration applied\r\n");
            }
            else
            {
                EP_CFG_LOG_INFO("NDEF record: Invalid KIWI data\r\n");
            }
        }
        else
        {
            EP_CFG_LOG_INFO("NDEF record: Invalid\r\n");
        }

exit:
    return;
}

static int ndef_validate_kiwi_data(nfc_ndef_bin_payload_desc_t *bin_payload_desc,
                                   nfc_ndef_record_desc_t *record_desc,
                                   nfc_ndef_record_location_t *record_location)
{
    int is_valid_kiwi_data = -1;

    if ( (*record_location == NDEF_LONE_RECORD)
      && (record_desc->tnf == TNF_EXTERNAL_TYPE)
      && (record_desc->id_length == 0)
      && (record_desc->type_length == sizeof(ndef_rec_field_type__kiwi))
      && (0 == memcmp(record_desc->p_type, ndef_rec_field_type__kiwi, sizeof(ndef_rec_field_type__kiwi))) )
    {
        if (bin_payload_desc->payload_length == sizeof(ep_cfg))
        {
            memcpy(&ep_cfg, bin_payload_desc->p_payload, sizeof(ep_cfg));

            uint16_t crc = crc16((char *)bin_payload_desc->p_payload, sizeof(ep_cfg) - sizeof(ep_cfg.crc));
            if (crc == REV16(ep_cfg.crc))
            {
                is_valid_kiwi_data = 0;
            }
            else
            {
                /* clear configuration structure to no use invalid data by accident */
                memset(&ep_cfg,0, sizeof(ep_cfg));
            }
        }
    }

    return is_valid_kiwi_data;
}

static __INLINE uint16_t REV16(uint16_t x)
{
    return (((x & 0x00ffU) << 8) & 0xff00) | (((x & 0xff00U) >> 8) & 0x00ff);
}

/* Function for calculating CRC16 is taken from:
 * https://twobora.atlassian.net/wiki/spaces/AD/pages/52330517/Zawarto+ci+pliku+NDEF */
#define POLY 0x8408
/*
//                                      16   12   5
// this is the CCITT CRC 16 polynomial X  + X  + X  + 1.
// This works out to be 0x1021, but the way the algorithm works
// lets us use 0x8408 (the reverse of the bit pattern).  The high
// bit is always assumed to be set, thus we only use 16 bits to
// represent the 17 bit value.
*/
static unsigned short crc16(char *data_p, unsigned short length)
{
      unsigned char i;
      unsigned int data;
      unsigned int crc = 0xffff;
      if (length == 0)
            return (~crc);
      do
      {
            for (i=0, data=(unsigned int)0xff & *data_p++;
                 i < 8;
                 i++, data >>= 1)
            {
                  if ((crc & 0x0001) ^ (data & 0x0001))
                        crc = (crc >> 1) ^ POLY;
                  else  crc >>= 1;
            }
      } while (--length);
      crc = ~crc;
      data = crc;
      crc = (crc << 8) | (data >> 8 & 0xff);
      return (crc);
}

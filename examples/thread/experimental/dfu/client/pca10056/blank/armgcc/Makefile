PROJECT_NAME     := thread_secure_dfu_client_pca10056
TARGETS          := nrf52840_xxaa
OUTPUT_DIRECTORY := _build

SDK_ROOT := ../../../../../../../..
PROJ_DIR := ../../..

$(OUTPUT_DIRECTORY)/nrf52840_xxaa.out: \
  LINKER_SCRIPT  := $(SDK_ROOT)/external/openthread/linker_scripts/openthread_nrf52840_dfu.ld

# Source files common to all targets
SRC_FILES += \
  $(SDK_ROOT)/components/libraries/bsp/bsp.c \
  $(SDK_ROOT)/components/libraries/bsp/bsp_nfc.c \
  $(SDK_ROOT)/components/libraries/bsp/experimental/bsp_thread.c \
  $(SDK_ROOT)/components/boards/boards.c \
  $(PROJ_DIR)/../../../../dfu/dfu_req_handling/dfu-cc.pb.c \
  $(PROJ_DIR)/../../../../dfu/dfu_req_handling/dfu_public_key.c \
  $(PROJ_DIR)/../../../../dfu/dfu_req_handling/dfu_req_handling.c \
  $(SDK_ROOT)/components/libraries/button/app_button.c \
  $(SDK_ROOT)/components/libraries/util/app_error.c \
  $(SDK_ROOT)/components/libraries/util/app_error_weak.c \
  $(SDK_ROOT)/components/libraries/scheduler/app_scheduler.c \
  $(SDK_ROOT)/components/libraries/timer/app_timer.c \
  $(SDK_ROOT)/components/libraries/util/app_util_platform.c \
  $(SDK_ROOT)/components/libraries/crc32/crc32.c \
  $(SDK_ROOT)/components/libraries/mem_manager/mem_manager.c \
  $(SDK_ROOT)/components/libraries/util/nrf_assert.c \
  $(SDK_ROOT)/components/libraries/queue/nrf_queue.c \
  $(SDK_ROOT)/components/libraries/strerror/nrf_strerror.c \
  $(SDK_ROOT)/components/libraries/sha256/sha256.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_serial.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_frontend.c \
  $(SDK_ROOT)/components/drivers_nrf/clock/nrf_drv_clock.c \
  $(SDK_ROOT)/components/drivers_nrf/common/nrf_drv_common.c \
  $(SDK_ROOT)/components/drivers_nrf/gpiote/nrf_drv_gpiote.c \
  $(SDK_ROOT)/components/drivers_nrf/rng/nrf_drv_rng.c \
  $(SDK_ROOT)/components/drivers_nrf/nrf_soc_nosd/nrf_nvic.c \
  $(SDK_ROOT)/components/drivers_nrf/hal/nrf_nvmc.c \
  $(SDK_ROOT)/components/drivers_nrf/nrf_soc_nosd/nrf_soc.c \
  $(SDK_ROOT)/components/libraries/crypto/nrf_crypto_ecdh.c \
  $(SDK_ROOT)/components/libraries/crypto/nrf_crypto_ecdsa.c \
  $(SDK_ROOT)/components/libraries/crypto/nrf_crypto_hash.c \
  $(SDK_ROOT)/components/libraries/crypto/nrf_crypto_init.c \
  $(SDK_ROOT)/components/libraries/crypto/nrf_crypto_keys.c \
  $(SDK_ROOT)/components/libraries/crypto/nrf_crypto_mem.c \
  $(SDK_ROOT)/components/libraries/crypto/nrf_crypto_rng.c \
  $(PROJ_DIR)/coap_dfu.c \
  $(PROJ_DIR)/main.c \
  $(SDK_ROOT)/external/segger_rtt/RTT_Syscalls_GCC.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_printf.c \
  $(PROJ_DIR)/../coap/coap.c \
  $(PROJ_DIR)/../coap/coap_block.c \
  $(PROJ_DIR)/../coap/coap_message.c \
  $(PROJ_DIR)/../coap/coap_option.c \
  $(PROJ_DIR)/../coap/coap_queue.c \
  $(PROJ_DIR)/../coap/coap_resource.c \
  $(PROJ_DIR)/../coap/coap_transport_ot.c \
  $(SDK_ROOT)/external/nano-pb/pb_common.c \
  $(SDK_ROOT)/external/nano-pb/pb_decode.c \
  $(SDK_ROOT)/components/toolchain/gcc/gcc_startup_nrf52840.S \
  $(SDK_ROOT)/components/toolchain/system_nrf52840.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/micro_ecc/micro_ecc_lib_ecdh.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/micro_ecc/micro_ecc_lib_ecdsa.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/micro_ecc/micro_ecc_lib_init.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/micro_ecc/micro_ecc_lib_keys.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/micro_ecc/micro_ecc_lib_shared.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/nrf_crypto_sw/nrf_crypto_sw_hash.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/nrf_crypto_sw/nrf_crypto_sw_rng.c \
  $(SDK_ROOT)/components/libraries/bootloader/dfu/nrf_dfu.c \
  $(SDK_ROOT)/components/libraries/bootloader/dfu/nrf_dfu_flash.c \
  $(SDK_ROOT)/components/libraries/bootloader/dfu/nrf_dfu_handling_error.c \
  $(SDK_ROOT)/components/libraries/bootloader/dfu/nrf_dfu_mbr.c \
  $(SDK_ROOT)/components/libraries/bootloader/dfu/nrf_dfu_settings.c \
  $(SDK_ROOT)/components/libraries/bootloader/dfu/nrf_dfu_transport.c \
  $(SDK_ROOT)/components/libraries/bootloader/dfu/nrf_dfu_utils.c \

# Include folders common to all targets
INC_FOLDERS += \
  $(SDK_ROOT)/components/libraries/svc \
  $(PROJ_DIR) \
  $(SDK_ROOT)/external/micro-ecc/micro-ecc \
  $(SDK_ROOT)/components/drivers_nrf/rng \
  $(SDK_ROOT)/components/toolchain/gcc \
  $(SDK_ROOT)/components/device \
  $(SDK_ROOT)/components/libraries/scheduler \
  $(SDK_ROOT)/components/drivers_nrf/hal \
  $(SDK_ROOT)/components/libraries/experimental_section_vars \
  $(SDK_ROOT)/components/libraries/crc32 \
  $(SDK_ROOT)/external/openthread/include \
  $(SDK_ROOT)/components/libraries/crypto/backend/micro_ecc \
  $(SDK_ROOT)/components/libraries/queue \
  $(SDK_ROOT)/components/libraries/strerror \
  $(SDK_ROOT)/components/boards \
  $(SDK_ROOT)/components/libraries/crypto/backend/nrf_crypto_sw \
  $(SDK_ROOT)/components/drivers_nrf/gpiote \
  $(PROJ_DIR)/../coap \
  $(SDK_ROOT)/components/drivers_nrf/nrf_soc_nosd \
  $(PROJ_DIR)/../../../../dfu/dfu_req_handling \
  $(SDK_ROOT)/components/libraries/button \
  $(SDK_ROOT)/components/libraries/log \
  $(SDK_ROOT)/components/libraries/log/src \
  $(SDK_ROOT)/components/libraries/bsp/experimental \
  $(SDK_ROOT)/components/libraries/bsp \
  $(SDK_ROOT)/components/drivers_nrf/clock \
  $(PROJ_DIR)/../mbr \
  ../config \
  $(SDK_ROOT)/external/nano-pb \
  $(SDK_ROOT)/components/libraries/bootloader \
  $(SDK_ROOT)/components/libraries/util \
  $(SDK_ROOT)/components/libraries/crypto \
  $(SDK_ROOT)/components/drivers_nrf/common \
  $(SDK_ROOT)/components \
  $(SDK_ROOT)/external/segger_rtt \
  $(SDK_ROOT)/components/drivers_nrf/delay \
  $(SDK_ROOT)/components/libraries/sha256 \
  $(SDK_ROOT)/components/toolchain \
  $(SDK_ROOT)/components/libraries/bootloader/dfu \
  $(SDK_ROOT)/components/toolchain/cmsis/include \
  $(SDK_ROOT)/components/libraries/mem_manager \
  $(SDK_ROOT)/external/nrf_cc310/include \
  $(SDK_ROOT)/components/libraries/fstorage \
  $(SDK_ROOT)/components/libraries/timer \

# Libraries common to all targets
LIB_FILES += \
  $(SDK_ROOT)/external/openthread/lib/gcc/libopenthread-cli-ftd.a \
  $(SDK_ROOT)/external/openthread/lib/gcc/libopenthread-ftd.a \
  $(SDK_ROOT)/external/openthread/lib/gcc/libopenthread-nrf52840-sdk.a \
  $(SDK_ROOT)/external/openthread/lib/gcc/libopenthread-diag.a \
  $(SDK_ROOT)/external/openthread/lib/gcc/libmbedcrypto.a \
  $(SDK_ROOT)/external/nrf_cc310/lib/libcc310_gcc_0.9.0.a \
  $(SDK_ROOT)/external/micro-ecc/nrf52_armgcc/armgcc/micro_ecc_lib_nrf52.a \

# C flags common to all targets
CFLAGS += -DBOARD_PCA10056
CFLAGS += -DCODE_REGION_1_START=0x1f000
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DMAIN_APPLICATION_START_ADDR=0x1f000
CFLAGS += -DNRF52840_XXAA
CFLAGS += -DNRF_DFU_NO_TRANSPORT
CFLAGS += -DNRF_DFU_SETTINGS_VERSION=1
CFLAGS += -DSVC_INTERFACE_CALL_AS_NORMAL_FUNCTION
CFLAGS += -DSWI_DISABLE0
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS +=  -Wall -Werror -O3 -g3
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums 

# C++ flags common to all targets
CXXFLAGS += \

# Assembler flags common to all targets
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DBOARD_PCA10056
ASMFLAGS += -DCODE_REGION_1_START=0x1f000
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DMAIN_APPLICATION_START_ADDR=0x1f000
ASMFLAGS += -DNRF52840_XXAA
ASMFLAGS += -DNRF_DFU_NO_TRANSPORT
ASMFLAGS += -DNRF_DFU_SETTINGS_VERSION=1
ASMFLAGS += -DSVC_INTERFACE_CALL_AS_NORMAL_FUNCTION
ASMFLAGS += -DSWI_DISABLE0

# Linker flags
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys


.PHONY: $(TARGETS) default all clean help flash 

# Default target - first one defined
default: nrf52840_xxaa

# Print all targets that can be built
help:
	@echo following targets are available:
	@echo 	nrf52840_xxaa

TEMPLATE_PATH := $(SDK_ROOT)/components/toolchain/gcc

include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))

# Flash the program
flash: $(OUTPUT_DIRECTORY)/nrf52840_xxaa.hex
	@echo Flashing: $<
	nrfjprog --program $< -f nrf52 --sectorerase
	nrfjprog --reset -f nrf52

erase:
	nrfjprog --eraseall -f nrf52

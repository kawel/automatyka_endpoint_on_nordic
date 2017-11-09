PROJECT_NAME     := thread_secure_dfu_bootloader_secure_dfu_blank_pca10056
TARGETS          := nrf52840_xxaa_blank
OUTPUT_DIRECTORY := _build

SDK_ROOT := ../../../../../../../..
PROJ_DIR := ../../..

$(OUTPUT_DIRECTORY)/nrf52840_xxaa_blank.out: \
  LINKER_SCRIPT  := $(PROJ_DIR)/thread_dfu_bootloader.ld

# Source files common to all targets
SRC_FILES += \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_serial.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_frontend.c \
  $(SDK_ROOT)/components/boards/boards.c \
  $(SDK_ROOT)/components/libraries/button/app_button.c \
  $(SDK_ROOT)/components/libraries/util/app_error_weak.c \
  $(SDK_ROOT)/components/libraries/scheduler/app_scheduler.c \
  $(SDK_ROOT)/components/libraries/timer/app_timer.c \
  $(SDK_ROOT)/components/libraries/util/app_util_platform.c \
  $(SDK_ROOT)/components/libraries/crc32/crc32.c \
  $(SDK_ROOT)/components/libraries/util/nrf_assert.c \
  $(SDK_ROOT)/components/libraries/strerror/nrf_strerror.c \
  $(SDK_ROOT)/components/drivers_nrf/common/nrf_drv_common.c \
  $(SDK_ROOT)/components/drivers_nrf/gpiote/nrf_drv_gpiote.c \
  $(SDK_ROOT)/components/drivers_nrf/nrf_soc_nosd/nrf_nvic.c \
  $(SDK_ROOT)/components/drivers_nrf/hal/nrf_nvmc.c \
  $(SDK_ROOT)/components/drivers_nrf/nrf_soc_nosd/nrf_soc.c \
  $(SDK_ROOT)/components/libraries/bsp/bsp.c \
  $(SDK_ROOT)/components/libraries/bsp/bsp_nfc.c \
  $(PROJ_DIR)/main.c \
  $(SDK_ROOT)/external/segger_rtt/RTT_Syscalls_GCC.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_printf.c \
  $(SDK_ROOT)/components/toolchain/gcc/gcc_startup_nrf52840.S \
  $(SDK_ROOT)/components/toolchain/system_nrf52840.c \
  $(SDK_ROOT)/components/libraries/bootloader/nrf_bootloader.c \
  $(SDK_ROOT)/components/libraries/bootloader/nrf_bootloader_app_start_asm.c \
  $(SDK_ROOT)/components/libraries/bootloader/nrf_bootloader_app_start_nosd.c \
  $(SDK_ROOT)/components/libraries/bootloader/nrf_bootloader_info.c \
  $(SDK_ROOT)/components/libraries/bootloader/dfu/nrf_dfu.c \
  $(SDK_ROOT)/components/libraries/bootloader/dfu/nrf_dfu_flash.c \
  $(SDK_ROOT)/components/libraries/bootloader/dfu/nrf_dfu_handling_error.c \
  $(SDK_ROOT)/components/libraries/bootloader/dfu/nrf_dfu_mbr.c \
  $(SDK_ROOT)/components/libraries/bootloader/dfu/nrf_dfu_settings.c \
  $(SDK_ROOT)/components/libraries/bootloader/dfu/nrf_dfu_transport.c \
  $(SDK_ROOT)/components/libraries/bootloader/dfu/nrf_dfu_utils.c \

# Include folders common to all targets
INC_FOLDERS += \
  $(PROJ_DIR) \
  $(SDK_ROOT)/components/toolchain/gcc \
  $(SDK_ROOT)/components/device \
  $(SDK_ROOT)/components/libraries/scheduler \
  $(SDK_ROOT)/components/drivers_nrf/hal \
  $(SDK_ROOT)/components/libraries/experimental_section_vars \
  $(SDK_ROOT)/components/libraries/crc32 \
  $(SDK_ROOT)/components/libraries/strerror \
  $(SDK_ROOT)/components/boards \
  $(SDK_ROOT)/components/drivers_nrf/gpiote \
  $(SDK_ROOT)/components/drivers_nrf/nrf_soc_nosd \
  $(SDK_ROOT)/components/libraries/button \
  $(SDK_ROOT)/components/libraries/log \
  $(SDK_ROOT)/components/libraries/log/src \
  $(SDK_ROOT)/components/toolchain \
  $(SDK_ROOT)/components/libraries/bsp \
  $(SDK_ROOT)/components/drivers_nrf/clock \
  $(PROJ_DIR)/../mbr \
  ../config \
  $(SDK_ROOT)/components/libraries/bootloader \
  $(SDK_ROOT)/components/libraries/util \
  $(SDK_ROOT)/components/drivers_nrf/common \
  $(SDK_ROOT)/components \
  $(SDK_ROOT)/external/segger_rtt \
  $(SDK_ROOT)/components/drivers_nrf/delay \
  $(SDK_ROOT)/components/libraries/bootloader/dfu \
  $(SDK_ROOT)/components/toolchain/cmsis/include \
  $(SDK_ROOT)/components/libraries/fstorage \
  $(SDK_ROOT)/components/libraries/timer \

# Libraries common to all targets
LIB_FILES += \

# C flags common to all targets
CFLAGS += -DBOARD_PCA10056
CFLAGS += -DCODE_REGION_1_START=0x1f000
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DMAIN_APPLICATION_START_ADDR=0x1f000
CFLAGS += -DNRF52840_XXAA
CFLAGS += -DNRF_DFU_SETTINGS_VERSION=1
CFLAGS += -DSWI_DISABLE0
CFLAGS += -D__HEAP_SIZE=0
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS +=  -Wall -Werror -Os -g3
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums -flto

# C++ flags common to all targets
CXXFLAGS += \

# Assembler flags common to all targets
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DBOARD_PCA10056
ASMFLAGS += -DCODE_REGION_1_START=0x1f000
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DMAIN_APPLICATION_START_ADDR=0x1f000
ASMFLAGS += -DNRF52840_XXAA
ASMFLAGS += -DNRF_DFU_SETTINGS_VERSION=1
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -D__HEAP_SIZE=0

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
default: nrf52840_xxaa_blank

# Print all targets that can be built
help:
	@echo following targets are available:
	@echo 	nrf52840_xxaa_blank

TEMPLATE_PATH := $(SDK_ROOT)/components/toolchain/gcc

include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))

# Flash the program
flash: $(OUTPUT_DIRECTORY)/nrf52840_xxaa_blank.hex
	@echo Flashing: $<
	nrfjprog --program $< -f nrf52 --sectorerase
	nrfjprog --reset -f nrf52

erase:
	nrfjprog --eraseall -f nrf52

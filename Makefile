# the1Comm (Tuya channel). Sources are flat - every .cpp/.hpp sits next to
# this file, there is no src/ or include/ sub-directory.

ROOT_DIR ?= $(abspath ../../)
-include $(ROOT_DIR)/build/build_param

OUTPUT_DIR = $(ROOT_DIR)/$(APP_PATH)/output/$(APP_NAME)_$(APP_VER)

TUYAOS_LIB_DIR = $(ROOT_DIR)/libs

APP_CFLAGS = -I$(ROOT_DIR)/$(APP_PATH)

LINKFLAGS = -L$(TUYAOS_LIB_DIR) \
            -Xlinker "-(" \
            -l$(APP_NAME) \
            -ltuyaapp_components \
            -ltuyaapp_drivers \
            -ltuyaos \
            -ltuyaos_adapter \
            -Xlinker "-)" \
            -pthread -lm

# BlueZ / D-Bus shared libraries, taken from the toolchain sysroot.
TOOLCHAIN_SYSROOT := $(ROOT_DIR)/vendor/aarch64-none-linux-gnu-gcc-11.2.1/toolchain/gcc-arm-11.2-2022.02-x86_64-aarch64-none-linux-gnu/aarch64-none-linux-gnu/libc
LINKFLAGS += -L$(TOOLCHAIN_SYSROOT)/usr/lib/aarch64-linux-gnu

LINKFLAGS += -Wl,--start-group \
              -ldbus-1 -lbluetooth -lglib-2.0 -lgio-2.0 -lgmodule-2.0 -lgobject-2.0 \
              -lpcre -lpcre2-8 -lmount -lblkid -luuid -lselinux -lffi -lz \
              -lsystemd -llzma -llz4 -lzstd -lcap -lgcrypt -lgpg-error \
              -Wl,--end-group

# The objects are C++ but the final link runs through $(CC); keep libstdc++ last.
LINKFLAGS += -lstdc++

all: app_excute

app_excute:
	@mkdir -p $(OUTPUT_DIR)
	$(CC) $(LINKFLAGS) -o $(OUTPUT_DIR)/$(APP_NAME)
	@echo "Build APP Finish"

.PHONY: all clean app_excute
clean:
	rm -rf $(OUTPUT_DIR)

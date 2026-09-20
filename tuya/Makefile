# 当前 Makefile 位于 apps/<app_name>/，SDK 根目录是 apps/ 的父目录
ROOT_DIR ?= $(abspath ../../)
-include $(ROOT_DIR)/build/build_param

# 编译所用变量
OUTPUT_DIR = $(ROOT_DIR)/$(APP_PATH)/output/$(APP_NAME)_$(APP_VER)

# 库文件路径
TUYAOS_LIB_DIR = $(ROOT_DIR)/libs

# 头文件路径
APP_CFLAGS = -I$(ROOT_DIR)/$(APP_PATH)/include

# 链接选项
LINKFLAGS = -L$(TUYAOS_LIB_DIR) \
            -Xlinker "-(" \
            -l$(APP_NAME) \
            -ltuyaapp_components \
            -ltuyaapp_drivers \
            -ltuyaos \
            -ltuyaos_adapter \
            -Xlinker "-)" \
            -pthread -lm

# BlueZ D-Bus 依赖的共享库（X3 板端需存在对应 .so）
TOOLCHAIN_SYSROOT := /home/by/桌面/X3M_TuyaOS-3.9.1/software/TuyaOS/vendor/aarch64-none-linux-gnu-gcc-11.2.1/toolchain/gcc-arm-11.2-2022.02-x86_64-aarch64-none-linux-gnu/aarch64-none-linux-gnu/libc
LINKFLAGS += -L$(TOOLCHAIN_SYSROOT)/usr/lib/aarch64-linux-gnu
# 用 --start-group/--end-group 避免库顺序问题
LINKFLAGS += -Wl,--start-group \
              -ldbus-1 -lbluetooth -lglib-2.0 -lgio-2.0 -lgmodule-2.0 -lgobject-2.0 \
              -lpcre -lpcre2-8 -lmount -lblkid -luuid -lselinux -lffi -lz \
              -lsystemd -llzma -llz4 -lzstd -lcap -lgcrypt -lgpg-error \
              -Wl,--end-group

all: app_excute

app_excute:
	@mkdir -p $(OUTPUT_DIR)
	$(CC) $(LINKFLAGS) -o $(OUTPUT_DIR)/$(APP_NAME)
	@echo "Build APP Finish"

.PHONY: all clean app_excute
clean:
	rm -rf $(OUTPUT_DIR)




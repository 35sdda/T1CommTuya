# Makefile for the1Comm（涂鸦通道版）
#
# 目录约定：**平铺** —— 所有 .cpp / .hpp / .md / .mk 与本文件同级，不分子目录。
#
# 相对原 demo app 的 Makefile 改了三处：
#   A. TOOLCHAIN_SYSROOT 原版写死了原开发者的机器路径 /home/by/桌面/...，
#      改成基于 ROOT_DIR，任何机器都能编。
#   B. 本 app 含 .cpp，最终链接必须带 -lstdc++。
#   C. 头文件搜索路径指向本目录自身（源文件之间 include 同目录的头文件）。

ROOT_DIR ?= $(abspath ../../)
-include $(ROOT_DIR)/build/build_param

# 编译所用变量
OUTPUT_DIR = $(ROOT_DIR)/$(APP_PATH)/output/$(APP_NAME)_$(APP_VER)

# 库文件路径
TUYAOS_LIB_DIR = $(ROOT_DIR)/libs

# 头文件路径（改动 C：平铺，直接 -I 本目录）
APP_CFLAGS = -I$(ROOT_DIR)/$(APP_PATH)

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

# BlueZ / D-Bus 依赖的共享库（X3 板端需存在对应 .so）
# 改动 A：改成基于 ROOT_DIR 的相对路径
TOOLCHAIN_SYSROOT := $(ROOT_DIR)/vendor/aarch64-none-linux-gnu-gcc-11.2.1/toolchain/gcc-arm-11.2-2022.02-x86_64-aarch64-none-linux-gnu/aarch64-none-linux-gnu/libc
LINKFLAGS += -L$(TOOLCHAIN_SYSROOT)/usr/lib/aarch64-linux-gnu

# 用 --start-group/--end-group 避免库顺序问题
LINKFLAGS += -Wl,--start-group \
              -ldbus-1 -lbluetooth -lglib-2.0 -lgio-2.0 -lgmodule-2.0 -lgobject-2.0 \
              -lpcre -lpcre2-8 -lmount -lblkid -luuid -lselinux -lffi -lz \
              -lsystemd -llzma -llz4 -lzstd -lcap -lgcrypt -lgpg-error \
              -Wl,--end-group

# 改动 B：本 app 的 .cpp 由 SDK 的 $(CXX) 编译，但下面这一行最终链接走的是 $(CC)。
# 不带 libstdc++ 会报 undefined reference to std::__cxx11::basic_string 之类。
# 放在最后，保证出现在所有库之后。
LINKFLAGS += -lstdc++

all: app_excute

app_excute:
	@mkdir -p $(OUTPUT_DIR)
	$(CC) $(LINKFLAGS) -o $(OUTPUT_DIR)/$(APP_NAME)
	@echo "Build APP Finish"

.PHONY: all clean app_excute
clean:
	rm -rf $(OUTPUT_DIR)

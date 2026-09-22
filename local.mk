# local.mk for the1Comm (Tuya channel). Sources are flat: LOCAL_PATH itself.

LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

# Module name == directory name == the1Comm -> libs/libthe1Comm.a, matching
# -l$(APP_NAME) in Makefile.
LOCAL_MODULE := $(notdir $(LOCAL_PATH))

LOCAL_SRC_FILES := $(wildcard $(LOCAL_PATH)/*.c) $(wildcard $(LOCAL_PATH)/*.cpp)

LOCAL_TUYA_SDK_INC := $(LOCAL_PATH)

# The SDK build system is C-only: for a .cpp it expands neither $(TARGET_CFLAGS)
# nor $(CXXFLAGS), and LOCAL_IMPORT_INCLUDES stays empty because this module
# declares no LOCAL_STATIC_LIBRARIES. The compile line then carries no SDK
# include path at all (tuya_cloud_types.h not found). LOCAL_CFLAGS is consumed
# through PRIVATE_CFLAGS for .c and .cpp alike, so the header trees go there.

TY_PLATFORM := $(if $(TARGET_PLATFORM),$(TARGET_PLATFORM),aarch64-none-linux-gnu-gcc-11.2.1)
TY_ADAPTER_INC := $(ROOT_DIR)/vendor/$(TY_PLATFORM)/tuyaos/tuyaos_adapter/include
TY_SYSROOT := $(ROOT_DIR)/vendor/$(TY_PLATFORM)/toolchain/gcc-arm-11.2-2022.02-x86_64-aarch64-none-linux-gnu/aarch64-none-linux-gnu/libc

TY_SDK_INC_DIRS := \
    $(shell find $(ROOT_DIR)/include -type d 2>/dev/null) \
    $(shell find $(TY_ADAPTER_INC) -type d 2>/dev/null) \
    $(TY_SYSROOT)/usr/include/glib-2.0 \
    $(TY_SYSROOT)/usr/include/glib-2.0/glib \
    $(TY_SYSROOT)/usr/include/glib-2.0/glib/deprecated \
    $(TY_SYSROOT)/usr/include/glib-2.0/gio \
    $(TY_SYSROOT)/usr/include/glib-2.0/gobject \
    $(TY_SYSROOT)/usr/lib/aarch64-linux-gnu/glib-2.0/include \
    $(TY_SYSROOT)/usr/include/dbus-1.0 \
    $(TY_SYSROOT)/usr/include/dbus-1.0/dbus \
    $(TY_SYSROOT)/usr/lib/aarch64-linux-gnu/dbus-1.0/include \
    $(TY_SYSROOT)/usr/lib/aarch64-linux-gnu/dbus-1.0/include/dbus \
    $(TY_SYSROOT)/usr/include/bluetooth \
    $(TY_SYSROOT)/usr/include/gdbus

LOCAL_CFLAGS := -I$(LOCAL_PATH) \
                $(addprefix -I,$(TY_SDK_INC_DIRS)) \
                -pipe -D_REENTRANT -Wall

TUYA_SDK_INC += $(LOCAL_TUYA_SDK_INC)

include $(BUILD_STATIC_LIBRARY)
include $(OUT_COMPILE_INFO)

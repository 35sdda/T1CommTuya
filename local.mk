# local.mk for the1Comm (Tuya-only the1Comm, phase 1)
#
# Layout: FLAT - every .cpp/.hpp sits next to this file, there is no src/ or
# include/ sub-directory (company convention).
#
# Differences vs the original demo app local.mk:
#   1. LOCAL_SRC_FILES must pick up *.cpp (the demo only globbed *.c), and the
#      sources live in LOCAL_PATH itself instead of LOCAL_PATH/src
#   2. LOCAL_CFLAGS must carry the SDK include paths, because the SDK xmake
#      does not pass them to .cpp compiles (see the big comment below)

LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

# Module name == directory name == the1Comm, produces libs/libthe1Comm.a,
# which matches -l$(APP_NAME) in Makefile.
LOCAL_MODULE := $(notdir $(LOCAL_PATH))

# Source files: flat layout, the sources are directly in LOCAL_PATH.
# The original demo only globbed *.c; this project is all C++, so *.cpp must be
# picked up as well, otherwise the files are silently skipped and the final link
# fails with undefined references.
LOCAL_SRC_FILES := $(wildcard $(LOCAL_PATH)/*.c) $(wildcard $(LOCAL_PATH)/*.cpp)

# Exported include dir: flat layout, the headers live in LOCAL_PATH itself
LOCAL_TUYA_SDK_INC := $(LOCAL_PATH)

# ============================================================================
# CRITICAL PATCH: hand the SDK include search paths to the C++ compile
#
# Symptom:
#     apps/the1Comm/src/T1Tuya.cpp:18:10: fatal error:
#         tuya_cloud_types.h: No such file or directory
#     compile command: g++ -Iapps/the1Comm -c ...
# i.e. the .cpp command line carried NO SDK include path at all, while the .c
# command line of the very same build carried dozens of -Ivendor/... -Iinclude/...
#
# Root cause in vendor/<platform>/tuyaos/build/xmake_binary.mk :
#     :97   $(LOCAL_INTERMEDIATE_OBJECTS): PRIVATE_CFLAGS := $(TARGET_CFLAGS) $(LOCAL_CFLAGS)
#     :124  %.c.o   rule -> build-object     , uses $(CFLAGS)   + PRIVATE_CFLAGS
#     :127  %.cpp.o rule -> build-cxx-object , uses $(CXXFLAGS) + PRIVATE_CFLAGS
# For this module TARGET_CFLAGS is empty (the .c side had -march=armv8-a etc,
# the .cpp side has nothing) and LOCAL_IMPORT_INCLUDES is empty too, because
# local.mk declares no LOCAL_STATIC_LIBRARIES and therefore inherits no
# export_includes from any dependency. Both empty => zero SDK include paths.
# Additionally $(CXXFLAGS) itself is empty: env.mk:45 only defines
# TARGET_CXXFLAGS = $(TARGET_CFLAGS), and build-cxx-object reads CXXFLAGS.
# In short: this SDK is C-only by design; mixing .cpp requires this patch.
#
# Fix: put the SDK header trees into LOCAL_CFLAGS. LOCAL_CFLAGS is reliably
# consumed through PRIVATE_CFLAGS, so this does not depend on SDK internals.
# It is also more robust than declaring LOCAL_STATIC_LIBRARIES, which relies on
# every SDK module exporting its include dirs correctly.
# ============================================================================

# Platform name from build_param when available, otherwise fall back.
TY_PLATFORM := $(if $(TARGET_PLATFORM),$(TARGET_PLATFORM),aarch64-none-linux-gnu-gcc-11.2.1)
TY_ADAPTER_INC := $(ROOT_DIR)/vendor/$(TY_PLATFORM)/tuyaos/tuyaos_adapter/include
TY_SYSROOT := $(ROOT_DIR)/vendor/$(TY_PLATFORM)/toolchain/gcc-arm-11.2-2022.02-x86_64-aarch64-none-linux-gnu/aarch64-none-linux-gnu/libc

# 151 dirs under include/ plus 29 dirs under tuyaos_adapter/include/.
# Covers tuya_cloud_*.h, tuya_iot_*.h, tuya_wifi_*.h, tuya_bt.h, tal_*.h,
# tuya_error_code.h, tuya_svc_upgrade.h, ws_db_gw.h, tuya_iot_config.h.
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

# -pipe -D_REENTRANT -Wall mirror what the SDK uses for .c compiles.
# The .cpp path goes through $(CXXFLAGS), which is empty, so carry them here.
# GCC 11 defaults to gnu++17 for C++, which is enough for this project.
LOCAL_CFLAGS := -I$(LOCAL_PATH) \
                $(addprefix -I,$(TY_SDK_INC_DIRS)) \
                -pipe -D_REENTRANT -Wall

# Global export, kept from the original demo local.mk
TUYA_SDK_INC += $(LOCAL_TUYA_SDK_INC)

include $(BUILD_STATIC_LIBRARY)
include $(OUT_COMPILE_INFO)

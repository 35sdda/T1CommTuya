# local.mk for tuyaos_demo_x3m_ble_wifi_netcfg

LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

# 模块名
LOCAL_MODULE := $(notdir $(LOCAL_PATH))

# 源文件
LOCAL_SRC_FILES := $(shell find $(LOCAL_PATH)/src -name "*.c")

# 对外头文件路径
LOCAL_TUYA_SDK_INC := $(LOCAL_PATH)/include

# 私有编译选项
LOCAL_CFLAGS := -I$(LOCAL_PATH)/include

# 使用 BlueZ D-Bus 时打开
# LOCAL_LDFLAGS += -ldbus-1

# 全局导出
TUYA_SDK_INC += $(LOCAL_TUYA_SDK_INC)

include $(BUILD_STATIC_LIBRARY)
include $(OUT_COMPILE_INFO)


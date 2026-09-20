/**
 * @file tuya_tkl_ble_adapter.h
 * @brief X3M Linux 平台 TKL 蓝牙适配层头文件
 */

#ifndef __TUYA_TKL_BLE_ADAPTER_H__
#define __TUYA_TKL_BLE_ADAPTER_H__

#include "tuya_cloud_types.h"
#include "tal_bluetooth_def.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 TKL 蓝牙适配层
 * @note  在调用 tal_ble_bt_init 之前由应用层调用，完成 BlueZ/D-Bus 等准备
 */
OPERATE_RET tuya_tkl_ble_adapter_init(VOID_T);

/**
 * @brief 反初始化 TKL 蓝牙适配层
 */
OPERATE_RET tuya_tkl_ble_adapter_deinit(VOID_T);

/**
 * @brief 获取当前保存的 TAL BLE 事件回调
 */
TAL_BLE_EVT_FUNC_CB tuya_tkl_ble_get_evt_cb(VOID_T);

#ifdef __cplusplus
}
#endif

#endif /* __TUYA_TKL_BLE_ADAPTER_H__ */

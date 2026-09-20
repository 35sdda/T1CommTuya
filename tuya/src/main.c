/**
 * @file main.c
 * @brief X3M TuyaOS BLE+WiFi 双模配网示例
 * @note  基于 TuyaOS 3.9.1 / tuyaos_robot_rlm_sdk
 *        适配 aarch64 Linux + BlueZ 蓝牙协议栈
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
 #include <sys/statfs.h>
#include <errno.h>
#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

#include "tuya_cloud_types.h"
#include "tuya_cloud_com_defs.h"
#include "tuya_cloud_wifi_defs.h"
#include "tuya_iot_com_api.h"
#include "tuya_iot_wifi_api.h"
#include "tuya_wifi_netcfg.h"
#include "tuya_wifi_reset.h"
#include "tuya_wifi_link.h"
#include "tuya_bt.h"
#include "tal_bluetooth.h"
#include "tal_system.h"
#include "tal_log.h"
#include "tuya_error_code.h"
#include "tuya_svc_upgrade.h"
//（头文件标注 "APIs should be implemented by developer"），不实现链接会报 undefined reference。
//本项目不用 SD 卡录像，所以用空实现（返回无卡/不写）满足链接器即可。 */
#include "ws_db_gw.h"
/* ========== [新增代码] 新增头文件（RLM 协议 + SD 卡适配） ========== */
#include "ty_rlm_func.h"
#include "tuya_ipc_stream_storage.h"
#include "tuya_ipc_stream_storage.h"
/* 为什么要适配 SD 卡：libtuyaos.a 的 IPC 组件要求开发者实现这几个接口
//（头文件标注 "APIs should be implemented by developer"），不实现链接会报 undefined reference。
// 本项目不用 SD 卡录像，所以用空实现（返回无卡/不写）满足链接器即可。 */
/* ========== [新增代码结束] ========== */
/* ============================================================
 *  涂鸦开发者平台授权信息
 *  量产时建议通过授权工具写入，调试阶段可直接写死或从命令行传入
 * ============================================================ */
#ifndef DEMO_PRODUCT_KEY
#define DEMO_PRODUCT_KEY    "lcuqw3h96rgtkhrz"
#endif

#ifndef DEMO_FIRMWARE_KEY
#define DEMO_FIRMWARE_KEY   DEMO_PRODUCT_KEY
#endif

#ifndef DEMO_WF_SW_VER
#define DEMO_WF_SW_VER      "1.0.0"
#endif

#ifndef DEMO_UUID
#define DEMO_UUID           "uuidfbc9b28086ee2c9a"
#endif

#ifndef DEMO_AUTH_KEY
#define DEMO_AUTH_KEY       "dCKOXmMhRbh8XeXGWKz0ZnFFjBmlHBYI"
#endif

#define USER_TUYA_PID      DEMO_PRODUCT_KEY
#define USER_TUYA_UUID     DEMO_UUID
#define USER_TUYA_AUTHKEY  DEMO_AUTH_KEY
#define USER_FIRMWARE_KEY  DEMO_FIRMWARE_KEY
#define USER_SW_VER        DEMO_WF_SW_VER

/* 持久化用户数据目录，OTA 重启后 KV/日志不能丢失。
 * 原先用 /userdata，现改为 /app/TheOne（你的实际挂载点）。 */
#define TY_APP_STORAGE_PATH    "/app/TheOne/tuya_x3m/"
#define TY_SDK_ONLINE_LOG_PATH "/app/TheOne/tuya_x3m/log/"

/* X3 开发板 Wi-Fi 网卡名称，需在 TKL Wi-Fi 适配层源码中使用（如 tal_wifi_adapter.c）。
 * 若 SDK 已自带适配层，请在该层把默认接口名改为 "wlan0"。 */
#define X3_WIFI_INTERFACE_NAME  "wlan0"

/* ============================================================
 *  OTA 文件存储路径
 * ============================================================ */
#define OTA_PACKAGE_TMP_PATH_DEFAULT   "/app/TheOne/Ver/ota_package.zip.tmp"
#define OTA_PACKAGE_FINAL_PATH_DEFAULT "/app/TheOne/Ver/ota_package.zip"
#define OTA_PACKAGE_TMP_PATH_FALLBACK  "/tmp/tuya_x3m/app/Version/ota_package.zip.tmp"
#define OTA_PACKAGE_FINAL_PATH_FALLBACK "/tmp/tuya_x3m/app/Version/ota_package.zip"

static CHAR_T g_ota_tmp_path[128]   = OTA_PACKAGE_TMP_PATH_DEFAULT;
static CHAR_T g_ota_final_path[128] = OTA_PACKAGE_FINAL_PATH_DEFAULT;

/* ============================================================
 *  回调函数
 * ============================================================ */
static VOID_T __gw_status_changed_cb(IN CONST GW_STATUS_E status)
{
    switch (status) {
    case GW_RESET:
        TAL_PR_NOTICE("gw status: reset");
        break;
    case GW_ACTIVED:
        TAL_PR_NOTICE("gw status: actived");
        break;
    case GW_FIRST_START:
        TAL_PR_NOTICE("gw status: first start");
        break;
    case GW_NORMAL:
        TAL_PR_NOTICE("gw status: normal");
        break;
    case GW_BLE_ACTIVED:
        TAL_PR_NOTICE("gw status: ble actived");
        break;
    default:
        TAL_PR_NOTICE("gw status: %d", status);
        break;
    }
}

static VOID_T __gw_reset_cb(IN CONST GW_RESET_TYPE_E type)
{
    TAL_PR_NOTICE("gw reset, type: %d", type);
}

/**
 * @brief OTA 升级前磁盘空间与业务保护检查
 */
static INT_T __ota_pre_check(IN CONST FW_UG_S *fw)
{
    struct statfs fs_info = {0};
    UINT64_T need_bytes = 0;
    UINT64_T free_bytes = 0;
    INT_T ret = OPRT_OK;

    TAL_PR_NOTICE("[OTA-CB] pre_check enter, fw=%p", fw);

    if (fw == NULL || fw->file_size == 0) {
        TAL_PR_ERR("ota pre_check invalid fw info");
        return OPRT_INVALID_PARM;
    }

    /* 1. 检查可用空间（固件大小 + 10MB 余量）。
     *    默认检查 /app/TheOne/Ver，不可用时回退到 /tmp。 */
    if (access("/app/TheOne/Ver", F_OK) == 0 && statfs("/app/TheOne/Ver", &fs_info) == 0) {
        TAL_PR_NOTICE("ota disk check on /app/TheOne/Ver");
    } else if (statfs("/tmp", &fs_info) == 0) {
        TAL_PR_NOTICE("ota disk check on /tmp (/app/TheOne/Ver unavailable)");
    } else {
        TAL_PR_ERR("ota statfs failed on both /app/TheOne/Ver and /tmp");
        return OPRT_COM_ERROR;
    }

    need_bytes = (UINT64_T)fw->file_size + 10 * 1024 * 1024;
    free_bytes = (UINT64_T)fs_info.f_bavail * (UINT64_T)fs_info.f_bsize;

    TAL_PR_NOTICE("ota disk check: need=%llu, free=%llu", need_bytes, free_bytes);

    if (free_bytes < need_bytes) {
        TAL_PR_ERR("ota disk space insufficient, refuse upgrade");
        ret = OPRT_COM_ERROR;
    }

    /* 2. TODO: 业务保护 —— 如有电机/加热等危险操作，先停止再允许升级 */
    /* if (is_device_running_critical_task()) { ret = OPRT_COM_ERROR; } */

    TAL_PR_NOTICE("[OTA-CB] pre_check exit, ret=%d", ret);
    return ret;
}

/* 前向声明：OTA 下载回调定义在 __gw_ug_cb 之后 */
static OPERATE_RET __ota_get_file_data_cb(IN CONST FW_UG_S *fw, IN CONST UINT_T total_len, IN CONST UINT_T offset,
                                          IN CONST BYTE_T *data, IN CONST UINT_T len, OUT UINT_T *remain_len, IN PVOID_T pri_data);
static OPERATE_RET __ota_upgrade_notify_cb(IN CONST FW_UG_S *fw, IN CONST INT_T download_result, IN PVOID_T pri_data);

/**
 * @brief 网关固件升级通知回调
 *
 * 返回 OPRT_OK 表示接受升级，SDK 将自动启动下载流程；
 * 返回其他值表示拒绝升级。
 */
static INT_T __gw_ug_cb(IN CONST FW_UG_S *fw)
{
    OPERATE_RET rt = OPRT_OK;

    TAL_PR_NOTICE("[OTA-CB] gw_ug_cb enter, fw=%p", fw);

    if (fw == NULL) {
        TAL_PR_ERR("gw upgrade inform, fw is null");
        return OPRT_INVALID_PARM;
    }

    TAL_PR_NOTICE("================================");
    TAL_PR_NOTICE("=== OTA Upgrade Inform ===");
    TAL_PR_NOTICE("version:      %s", fw->sw_ver);
    TAL_PR_NOTICE("file_size:    %u", fw->file_size);
    TAL_PR_NOTICE("diff_ota:     %d", fw->diff_ota);
    TAL_PR_NOTICE("upgrade_type: %d (0=normal,1=silent,2=lan)", fw->type);
    TAL_PR_NOTICE("================================");

    /* TODO: 业务保护 —— 如机器人正在运动/加热/充电等，可在此暂停或拒绝 */
    /* if (is_device_running_critical_task()) { return OPRT_COM_ERROR; } */

    /* 涂鸦网关/本机 OTA 都需要在通知回调里显式启动下载 */
    rt = tuya_iot_upgrade_gw_notify(fw, __ota_get_file_data_cb, __ota_upgrade_notify_cb,
                                    NULL, TRUE, 0);
    if (rt != OPRT_OK) {
        TAL_PR_ERR("[OTA-CB] gw tuya_iot_upgrade_gw_notify failed: %d", rt);
        return rt;
    }
    TAL_PR_NOTICE("[OTA-CB] gw tuya_iot_upgrade_gw_notify ok");

    return OPRT_OK;
}

/**
 * @brief 子设备/本机固件升级前检查（SOC 产品 OTA 通常走这里）
 *
 * 涂鸦平台对 SOC 产品通常下发“设备固件升级”，因此 pre_dev_ug_cb 必须实现。
 */
static INT_T __ota_pre_check_dev(IN CONST CHAR_T *dev_id, IN CONST FW_UG_S *fw)
{
    INT_T ret = OPRT_OK;

    TAL_PR_NOTICE("[OTA-CB] pre_check_dev enter, dev_id=%s, fw=%p",
                  dev_id ? dev_id : "(null)", fw);
    ret = __ota_pre_check(fw);
    TAL_PR_NOTICE("[OTA-CB] pre_check_dev exit, ret=%d", ret);
    return ret;
}

/**
 * @brief 子设备/本机固件升级通知回调
 */
static INT_T __dev_ug_cb(IN CONST CHAR_T *dev_id, IN CONST FW_UG_S *fw)
{
    TAL_PR_NOTICE("[OTA-CB] dev_ug_cb enter, dev_id=%s, fw=%p",
                  dev_id ? dev_id : "(null)", fw);

    if (fw == NULL) {
        TAL_PR_ERR("dev upgrade inform, fw is null");
        return OPRT_INVALID_PARM;
    }

    TAL_PR_NOTICE("================================");
    TAL_PR_NOTICE("=== Dev OTA Upgrade Inform ===");
    TAL_PR_NOTICE("dev_id:       %s", dev_id ? dev_id : "gw");
    TAL_PR_NOTICE("version:      %s", fw->sw_ver);
    TAL_PR_NOTICE("file_size:    %u", fw->file_size);
    TAL_PR_NOTICE("diff_ota:     %d", fw->diff_ota);
    TAL_PR_NOTICE("upgrade_type: %d (0=normal,1=silent,2=lan)", fw->type);
    TAL_PR_NOTICE("================================");

    TAL_PR_NOTICE("[OTA-CB] dev_ug_cb exit, ret=0");
    return OPRT_OK;
}

/* ============================================================
 *  tuya_svc_upgrade_init 专用回调（SOC 产品真正生效的 OTA 入口）
 * ============================================================
 *  涂鸦 SDK 在 tuya_svc_upgrade 内部使用 TUYA_UPGRADE_PARAM_T 结构体，
 *  其中 dev_upgrade_cb / pre_dev_upgrade_cb 的签名为：
 *      INT_T (*)(CONST FW_UG_S *fw)
 *  与 TY_IOT_CBS_S.pre_dev_ug_cb 的签名（带 dev_id）不同，
 *  因此仅注册 TY_IOT_CBS_S 会导致 "pre dev ug check failed"。
 */

/**
 * @brief OTA 下载数据通知回调
 *
 * tuya_iot_upgrade_gw_notify 走应用层下载路径，数据通过本回调交给应用层。
 * 这里把每片数据追加写入临时文件，由 __ota_upgrade_notify_cb 在下载完成
 * 后做原子重命名。
 */
static FILE *g_ota_fp = NULL;

static OPERATE_RET __ota_get_file_data_cb(IN CONST FW_UG_S *fw, IN CONST UINT_T total_len, IN CONST UINT_T offset,
                                          IN CONST BYTE_T *data, IN CONST UINT_T len, OUT UINT_T *remain_len, IN PVOID_T pri_data)
{
    size_t written = 0;

    *remain_len = 0;

    if (fw == NULL || data == NULL || len == 0) {
        TAL_PR_WARN("[OTA-CB] get_file_data invalid param, fw=%p data=%p len=%u", fw, data, len);
        return OPRT_INVALID_PARM;
    }

    /* 首包：创建目录并打开临时文件 */
    if (offset == 0 || g_ota_fp == NULL) {
        if (g_ota_fp) {
            fclose(g_ota_fp);
            g_ota_fp = NULL;
        }

        mkdir("/app", 0755);
        mkdir("/app/TheOne", 0755);
        mkdir("/app/TheOne/Ver", 0755);
        mkdir("/userdata", 0755);
        mkdir("/userdata/app", 0755);
        mkdir("/userdata/app/Version", 0755);
        mkdir("/tmp/tuya_x3m", 0755);
        mkdir("/tmp/tuya_x3m/app", 0755);
        mkdir("/tmp/tuya_x3m/app/Version", 0755);
        unlink(g_ota_tmp_path);

        g_ota_fp = fopen(g_ota_tmp_path, "wb");
        if (g_ota_fp == NULL) {
            TAL_PR_ERR("[OTA-CB] open ota tmp file failed: %s", strerror(errno));
            return OPRT_COM_ERROR;
        }

        TAL_PR_NOTICE("[OTA-CB] ota tmp file opened: %s, total_len=%u", g_ota_tmp_path, total_len);
    }

    written = fwrite(data, 1, len, g_ota_fp);
    if (written != len) {
        TAL_PR_ERR("[OTA-CB] ota write failed: written=%u, expect=%u", (UINT_T)written, len);
        fclose(g_ota_fp);
        g_ota_fp = NULL;
        unlink(g_ota_tmp_path);
        return OPRT_COM_ERROR;
    }

    TAL_PR_NOTICE("[OTA-CB] ota recv progress: offset=%u, len=%u", offset, len);
    return OPRT_OK;
}

/**
 * @brief OTA 下载完成/失败通知回调
 *
 * 涂鸦 App 会等待设备上报升级进度和结果。下载完成后必须主动上报：
 *  - 进度 100%
 *  - 结果 TUS_UPGRD_FINI（完成）或 TUS_UPGRD_EXEC（失败）
 * 否则 App 会卡在"更新中..."直到超时。
 */
static OPERATE_RET __ota_upgrade_notify_cb(IN CONST FW_UG_S *fw, IN CONST INT_T download_result, IN PVOID_T pri_data)
{
    OPERATE_RET rt = OPRT_OK;
    INT_T report_result = TUS_UPGRD_EXEC;

    TAL_PR_NOTICE("[OTA-CB] upgrade notify, result=%d", download_result);
    if (fw) {
        TAL_PR_NOTICE("fw_url:    %s", fw->fw_url);
        TAL_PR_NOTICE("sw_ver:    %s", fw->sw_ver);
        TAL_PR_NOTICE("file_size: %u", fw->file_size);
    }

    if (g_ota_fp) {
        fclose(g_ota_fp);
        g_ota_fp = NULL;
    }

    if (download_result == 0) {
        if (rename(g_ota_tmp_path, g_ota_final_path) != 0) {
            TAL_PR_ERR("[OTA-CB] ota rename failed: %s", strerror(errno));
            report_result = TUS_UPGRD_EXEC;
        } else {
            TAL_PR_NOTICE("[OTA-CB] ota package ready: %s", g_ota_final_path);
            report_result = TUS_UPGRD_FINI;
        }
    } else {
        TAL_PR_ERR("[OTA-CB] ota download failed, remove tmp file");
        unlink(g_ota_tmp_path);
        report_result = TUS_UPGRD_EXEC;
    }

    /* 上报升级进度 100% 和结果，否则 App 一直显示"更新中" */
    rt = tuya_iot_dev_upgd_progress_rept(100, NULL, DEV_NM_ATH_SNGL);
    TAL_PR_NOTICE("[OTA-CB] report progress 100% ret=%d", rt);

    rt = tuya_iot_dev_upgd_result_report(NULL, DEV_NM_ATH_SNGL, report_result);
    TAL_PR_NOTICE("[OTA-CB] report result %d ret=%d", report_result, rt);

    return OPRT_OK;
}

/**
 * @brief SOC 本机升级通知回调（供 tuya_svc_upgrade_init 使用）
 *
 * 涂鸦 SOC OTA 标准流程：在此回调中调用 tuya_iot_upgrade_gw_notify
 * 主动启动固件下载，否则 SDK 不会进入下载阶段。
 */
static INT_T __dev_upgrade_inform_cb(CONST FW_UG_S *fw)
{
    OPERATE_RET rt = OPRT_OK;

    TAL_PR_NOTICE("[OTA-CB] dev_upgrade_inform_cb enter, fw=%p", fw);

    if (fw == NULL) {
        TAL_PR_ERR("dev upgrade inform, fw is null");
        return OPRT_INVALID_PARM;
    }

    TAL_PR_NOTICE("================================");
    TAL_PR_NOTICE("=== OTA Upgrade Inform ===");
    TAL_PR_NOTICE("version:      %s", fw->sw_ver);
    TAL_PR_NOTICE("file_size:    %u", fw->file_size);
    TAL_PR_NOTICE("fw_url:       %s", fw->fw_url);
    TAL_PR_NOTICE("diff_ota:     %d", fw->diff_ota);
    TAL_PR_NOTICE("upgrade_type: %d (0=normal,1=silent,2=lan)", fw->type);
    TAL_PR_NOTICE("================================");

    /* 启动 SDK 下载流程；数据写入由 TKL 层 (tkl_ota.c) 完成。 */
    rt = tuya_iot_upgrade_gw_notify(fw, __ota_get_file_data_cb, __ota_upgrade_notify_cb,
                                    NULL, TRUE, 0);
    if (rt != OPRT_OK) {
        TAL_PR_ERR("tuya_iot_upgrade_gw_notify failed: %d", rt);
        return rt;
    }
    TAL_PR_NOTICE("[OTA-CB] tuya_iot_upgrade_gw_notify ok");

    return OPRT_OK;
}

/**
 * @brief SOC 本机升级前检查回调（供 tuya_svc_upgrade_init 使用）
 */
static INT_T __pre_dev_upgrade_cb(CONST FW_UG_S *fw)
{
    INT_T ret = OPRT_OK;

    TAL_PR_NOTICE("[OTA-CB] pre_dev_upgrade_cb enter, fw=%p", fw);
    ret = __ota_pre_check(fw);
    TAL_PR_NOTICE("[OTA-CB] pre_dev_upgrade_cb exit, ret=%d", ret);

    /* 涂鸦部分 OTA 预检查接口期望返回 TUS_RD(1) 表示允许升级，
     * 而不是 OPRT_OK(0)。保留 0 会导致 SDK 报 pre dev ug check failed。 */
    return (ret == OPRT_OK) ? TUS_RD : ret;
}

/* ========== [新增代码] DP 定义与处理（103~110、113） ========== */
/* ============================================================
 *  DP 定义与处理
 *  103~110 为普通 obj DP（bool/value），113 为设备状态（value）
 * ============================================================ */
#define DP_ID_MOWER_LIFE   101  /* 刀片寿命，value */
#define DP_ID_BLADE_LIFE   102  /* 切割盘寿命，value */
#define DP_ID_RAIN_EN      103  /* 雨天割草开关，bool */
#define DP_ID_EDGE_EN      104  /* 沿边割草开关，bool */
#define DP_ID_DELAY_TIME   105  /* 延迟割草时间，value */
#define DP_ID_EDGE_DIST    106  /* 沿边割草距离，value */
#define DP_ID_SWITCH_107   107  /* 预留开关（需求文档：随机割草开关），业务待协议确认 */
#define DP_ID_BT_EN        108  /* 蓝牙开关，bool */
#define DP_ID_SWITCH_109   109  /* 预留开关（需求文档：刺猬保护），业务待协议确认 */
#define DP_ID_SWITCH_110   110  /* 预留开关（需求文档：随机割草模式切换），业务待协议确认 */
#define DP_ID_DEV_STATUS   113  /* 设备状态，value：0=待机 1=出桩 2=回桩 3=作业 4=暂停 */
#define DP_ID_MOVE_DIR     121  /* 121：移动方向，value：0停止/1前进/2后退/3左转/4右转 */


/* 普通 obj DP 的缓存值，由 App 下发后更新 */
static BOOL_T g_rain_en=FALSE;  /* 103 雨天割草开关当前值，默认关 */
static BOOL_T g_edge_en=FALSE;  /* 104 沿边割草开关当前值 */
static INT_T g_delay_time=0;  /* 105 延迟时间（分钟），默认 0 */
static INT_T g_edge_dist=0;  /* 106 沿边割草距离，默认 0 */
static BOOL_T g_sw107=FALSE;  /* 107 预留开关当前值（业务待协议确认，先存着） */
static BOOL_T g_bt_en=TRUE;  /* 108 蓝牙开关，默认开 */
static BOOL_T g_sw109=FALSE;  /* 109 预留开关当前值（业务待协议确认，先存着） */
static BOOL_T g_sw110=FALSE;  /* 110 预留开关当前值（业务待协议确认，先存着） */
static INT_T g_dev_status=0;  /* 113 设备状态缓存，默认待机 */
static INT_T g_move_dir=0;     /* 121 移动方向当前值，默认停止 */

/* 前置声明：TheOne TCP 发送函数，定义在后面 RLM 区块 */
static OPERATE_RET send_ctrl_cmd_to_theone(CHAR_T cmd);

/**
 * @brief 处理一个 bool 类型的 DP 下发
 */
static VOID_T handle_bool_dp(IN CONST TY_OBJ_DP_S *one)  /* bool 型 DP 的统一处理函数，one 是单条 DP */
{
    switch ( one->dpid ) {  /* 按 dpid 分发到对应业务 */
    case DP_ID_RAIN_EN:  /* 103：雨天割草开关 */
        g_rain_en=one->value.dp_bool;  /* 保存 App 下发的开关值到全局变量 */
        TAL_PR_NOTICE("DP103 rain switch: %d", g_rain_en);  /* 打印日志，方便调试 */
        break;
    case DP_ID_EDGE_EN:  /* 104：沿边割草开关 */
        g_edge_en=one->value.dp_bool;  /* 保存开关值 */
        TAL_PR_NOTICE("DP104 edge switch: %d", g_edge_en);  /* 打印日志 */
        break;
    case DP_ID_SWITCH_107:  /* 107：预留开关 */
        g_sw107=one->value.dp_bool;  /* 保存开关值 */
        TAL_PR_NOTICE("DP107 switch: %d", g_sw107);  /* 打印日志 */
        break;
    case DP_ID_BT_EN:  /* 108：蓝牙开关 */
        g_bt_en=one->value.dp_bool;  /* 保存开关值 */
        tuya_ble_set_serv_switch(g_bt_en);  /* 立即生效 */
        TAL_PR_NOTICE("DP108 bluetooth switch: %d", g_bt_en);  /* 打印日志 */
        break;
    case DP_ID_SWITCH_109:  /* 109：预留开关 */
        g_sw109=one->value.dp_bool;  /* 保存开关值 */
        TAL_PR_NOTICE("DP109 switch: %d", g_sw109);  /* 打印日志 */
        break;
    case DP_ID_SWITCH_110:  /* 110：预留开关 */
        g_sw110=one->value.dp_bool;  /* 保存开关值 */
        TAL_PR_NOTICE("DP110 switch: %d", g_sw110);  /* 打印日志 */
        break;
    default:
        TAL_PR_NOTICE("unknown bool dp, dpid: %u", one->dpid);  /* 未知 bool DP，打印后忽略 */
        break;
    }
}

/**
 * @brief 处理一个 value 类型的 DP 下发
 */
static VOID_T handle_value_dp(IN CONST TY_OBJ_DP_S *one)  /* value 型 DP 的统一处理函数 */
{
    switch ( one->dpid ) {  /* 按 dpid 分发到对应业务 */
    case DP_ID_DELAY_TIME:  /* 105：延迟时间 */
        g_delay_time=one->value.dp_value;  /* 保存数值 */
        TAL_PR_NOTICE("DP105 delay time: %d", g_delay_time);  /* 打印日志 */
        break;
    case DP_ID_EDGE_DIST:  /* 106：沿边距离 */
        g_edge_dist=one->value.dp_value;  /* 保存数值 */
        TAL_PR_NOTICE("DP106 edge distance: %d", g_edge_dist);  /* 打印日志 */
        break;
    case DP_ID_MOVE_DIR:
        g_move_dir=one->value.dp_value;   /* 存方向值 */
        TAL_PR_NOTICE("DP121 move dir: %d", g_move_dir);
        switch ( g_move_dir ) {           /* 方向 → TheOne 指令 */
            case 1: send_ctrl_cmd_to_theone('w'); break;   /* 前进 */
            case 2: send_ctrl_cmd_to_theone('s'); break;   /* 后退 */
            case 3: send_ctrl_cmd_to_theone('a'); break;   /* 左转 */
            case 4: send_ctrl_cmd_to_theone('d'); break;   /* 右转 */
            default: send_ctrl_cmd_to_theone('g'); break;  /* 0/其他：停止 */
        }
        break;
    default:
        TAL_PR_NOTICE("unknown value dp, dpid: %u", one->dpid);  /* 未知 value DP，忽略 */
        break;
    }
}

/**
 * @brief 上报设备状态到云端（DP113），状态未变化时跳过
 */
static OPERATE_RET report_device_status(INT_T status)  /* 上报 DP113 设备状态，状态变化才上报 */
{
    if ( status==g_dev_status ) {  /* 状态没变就不重复上报 */
        return OPRT_OK;  /* 状态未变化，跳过上报 */
    }
    g_dev_status=status;  /* 更新本地状态缓存 */

    TAL_PR_NOTICE("DP113 report status: %d", status);

    TY_OBJ_DP_S dp={0};  /* 构造单条 DP 上报结构体 */
    dp.dpid=DP_ID_DEV_STATUS;  /* DP 编号 113 */
    dp.type=PROP_VALUE;  /* 类型：数值 */
    dp.value.dp_value=status;  /* 填入要上报的状态值 */
    dp.time_stamp=0;  /* 时间戳填 0，由 SDK 处理 */

    return dev_report_dp_json_async(NULL, &dp, 1);  /* NULL 表示上报本设备 */
}

static VOID_T __dev_obj_dp_cb(IN CONST TY_RECV_OBJ_DP_S *dp)  /* 普通 obj DP 回调：SDK 收到 App 下发后调用 */
{
    if ( !dp ) {  /* 空指针保护 */
        return;
    }
    TAL_PR_NOTICE("recv obj dp, cnt: %u", dp->dps_cnt);  /* 打印本次收到几条 DP */

    INT_T i=0;  /* 循环变量 */
    for ( i=0; i<dp->dps_cnt; i++ ) {  /* 一次可能下发多条，逐条处理 */
        CONST TY_OBJ_DP_S *one=&dp->dps[i];  /* 取第 i 条（注意类型是单条 DP，不是容器） */
        if ( one->type==PROP_BOOL ) {  /* bool 类型走 bool 处理函数 */
            handle_bool_dp(one);  /* 调用 bool 处理函数 */
        } else if ( one->type==PROP_VALUE ) {  /* value 类型走 value 处理函数 */
            handle_value_dp(one);  /* 调用 value 处理函数 */
        } else {
            TAL_PR_NOTICE("skip dp, dpid: %u type: %u", one->dpid, one->type);  /* 其他类型暂不处理 */
        }
    }
}

static VOID_T __dev_raw_dp_cb(IN CONST TY_RECV_RAW_DP_S *dp)  /* raw DP 回调：收到未解析的二进制数据 */
{
    if ( !dp ) {  /* 空指针保护 */
        return;
    }
    TAL_PR_NOTICE("recv raw dp, dpid: %u len: %u", dp->dpid, dp->len);  /* 打印 raw DP 编号和长度 */
    /* raw DP（111/112/114/150）暂不处理，待协议评审后补充 */
}

static VOID_T __dev_dp_query_cb(IN CONST TY_DP_QUERY_S *dp_qry)  /* App 主动查询 DP 时 SDK 调用 */
{
    if ( !dp_qry ) {
        return;
    }
    TAL_PR_NOTICE("recv dp query");  /* 打印查询事件 */
    /* TODO: App 查询时上报当前 DP 值 */
}
static VOID_T __wf_nw_stat_cb(IN CONST GW_WIFI_NW_STAT_E stat)  /* WiFi 网络状态变化回调 */
{
    TAL_PR_NOTICE("wifi nw stat: %d", stat);  /* 打印当前 WiFi 状态 */
}

static VOID_T __wf_cfg_err_code_cb(IN CONST NW_CFG_ERR_CODE_E err_code)  /* 配网错误码回调 */
{
    TAL_PR_NOTICE("wifi cfg err code: %d", err_code);  /* 打印配网错误码 */
}

// /* ========== [临时测试] 模拟云端下发 DP103=开 ========== */
// static VOID_T test_dp_downlink(VOID_T)
// {
//     TY_RECV_OBJ_DP_S dp={0};
//     TY_OBJ_DP_S one={0};

//     one.dpid=DP_ID_RAIN_EN;
//     one.type=PROP_BOOL;
//     one.value.dp_bool=TRUE;
//     dp.dps_cnt=1;
//     dp.dps[0]=one;

//     __dev_obj_dp_cb(&dp);
//     TAL_PR_NOTICE("test dp downlink done");
// }
// /* ========== [临时测试结束] ========== */

/* ========== [新增代码结束] ========== */

/* ========== [新增代码] RLM 复杂功能回调（割草/回充） ========== */
/* ============================================================
 *  RLM 复杂功能回调
 *  老型号协议中与新型号一致的任务控制部分
 * ============================================================ */

/* ============================================================
 *  TheOne TCP 客户端
 *  连接 127.0.0.1:1111，发送 $C:x 指令给 TheOne，
 *  接收 S: 心跳并解析状态，通过 DP113 上报到涂鸦云端
 * ============================================================ */
#define THEONE_HOST       "127.0.0.1"
#define THEONE_PORT       1111
#define THEONE_CMD_MAX    64
#define THEONE_RECV_MAX   512

/* 全局 socket 句柄，加锁防止发送和接收线程同时操作 */
static INT_T g_theone_sock=-1;
static pthread_mutex_t g_theone_lock=PTHREAD_MUTEX_INITIALIZER;

/**
 * @brief 连接 TheOne 的 1111 端口
 * @return 成功返回 socket 句柄，失败返回 -1
 */
static INT_T theone_connect(VOID_T)
{
    INT_T sock=socket(AF_INET, SOCK_STREAM, 0);

    if ( sock<0 ) {
        TAL_PR_ERR("theone socket failed");
        return -1;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family=AF_INET;
    addr.sin_port=htons(THEONE_PORT);
    inet_pton(AF_INET, THEONE_HOST, &addr.sin_addr);

    if ( connect(sock, (struct sockaddr *)&addr, sizeof(addr))!=0 ) {
        TAL_PR_ERR("theone connect failed: %s", strerror(errno));
        close(sock);
        return -1;
    }
    TAL_PR_NOTICE("theone connected: %s:%d", THEONE_HOST, THEONE_PORT);

    /* 设为非阻塞，recv 不会卡住等数据 */
    int flags=fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags|O_NONBLOCK);

    return sock;
}

/**
 * @brief 真正发送 $C:x 给 TheOne（替换原来的桩函数）
 * @param[in] cmd 指令字符，例如 '3'=开始割草 '0'=停止 'h'=回家
 * @return OPRT_OK 发送成功，其他为失败
 */
static OPERATE_RET send_ctrl_cmd_to_theone(CHAR_T cmd)
{
    CHAR_T buf[THEONE_CMD_MAX]={0};
    INT_T len=0;

    /* 拼出协议格式：$C:x\r\n，x 就是要执行的指令字符 */
    snprintf(buf, sizeof(buf), "$C:%c\r\n", cmd);
    pthread_mutex_lock(&g_theone_lock);
    if ( g_theone_sock<0 ) {
        TAL_PR_ERR("theone not connected, cmd dropped");
    } else {
        len=send(g_theone_sock, buf, strlen(buf), 0);
        TAL_PR_NOTICE("theone send: [%s] bytes=%d", buf, len);
    }
    pthread_mutex_unlock(&g_theone_lock);
    return ( len>0 ) ? OPRT_OK : OPRT_COM_ERROR;
}

/**
 * @brief 从 TheOne 心跳字符串里解析 S:<status>
 * @param[in] buf 收到的数据
 * @param[in] len 数据长度
 * @return 状态值：0=待机 1=出桩 2=回桩 3=作业 4=暂停，找不到返回 -1
 */
static INT_T theone_parse_status(const CHAR_T *buf, INT_T len)
{
    INT_T i=0;

    for ( i=0; i+1<len; i++ ) {
        if ( buf[i]=='S' && buf[i+1]==':' ) {
            return atoi(&buf[i+2]);
        }
    }
    return -1;
}

/**
 * @brief 心跳接收线程：断线自动重连，收到状态就上报 DP113
 */
static VOID_T *theone_recv_thread(VOID_T *arg)
{
    CHAR_T buf[THEONE_RECV_MAX]={0};
    INT_T n=0;
    INT_T status=-1;

    (VOID_T)arg;
    while ( 1 ) {
        pthread_mutex_lock(&g_theone_lock);
        if ( g_theone_sock<0 ) {
            g_theone_sock=theone_connect();
        }
        pthread_mutex_unlock(&g_theone_lock);

        if ( g_theone_sock<0 ) {
            sleep(3);   /* 连不上等 3 秒重试 */
            continue;
        }

        memset(buf, 0, sizeof(buf));
        n=recv(g_theone_sock, buf, sizeof(buf)-1, 0);
        if ( n<0 ) {
            if ( errno==EAGAIN || errno==EWOULDBLOCK ) {   /* 非阻塞，没数据就等一会 */
                usleep(10000);
                continue;
            }
        }
        if ( n<=0 ) {
            TAL_PR_WARN("theone recv closed, reconnect");
            close(g_theone_sock);
            g_theone_sock=-1;
            sleep(1);
            continue;
        }
        /* recv 收到的是原始字节，结尾没有 \0；
         * 手动补一个 \0，后面的 atoi 才能把 "S:3" 当字符串解析 */
        buf[n]='\0';
        /* 在 buf 里找 "S:" 并取出后面的数字，得到 0~4 的设备状态 */
        status=theone_parse_status(buf, n);
        if ( status>=0 ) {
            TAL_PR_NOTICE("theone heartbeat status: %d", status);
            report_device_status(status);   /* 状态变化才上报，函数内部有去重 */
        }
    }
    return NULL;
}

/**
 * @brief 启动 TheOne TCP 客户端线程
 *        pthread_create 4 个参数：
 *        1. &tid：输出参数，线程 ID 写到这里
 *        2. NULL：线程属性，用默认
 *        3. theone_recv_thread：新线程从这里开始执行
 *        4. NULL：传给线程函数的参数，这里没有
 *        返回 0 表示成功，非 0 表示创建失败（如内存不足）
 */
static VOID_T theone_start(VOID_T)
{
    pthread_t tid;   /* 线程句柄，只是接住 ID，后面不用 */

    if ( pthread_create(&tid, NULL, theone_recv_thread, NULL)!=0 ) {
        TAL_PR_ERR("theone thread create failed");
    }
}

/**
 * @brief RLM 设置命令回调，SDK 解析云端下发后调用
 */
static OPERATE_RET rlm_advance_function_set(IN COMPLEX_RLM_CMD_E cmd, IN void* param)  /* RLM SET 回调：云端下发命令时 SDK 调用 */
{
    OPERATE_RET ret=OPRT_OK;

    if ( !param ) {  /* 参数为空直接返回 */
        return OPRT_INVALID_PARM;  /* 返回参数错误 */
    }

    switch ( cmd ) {  /* 按命令类型分发 */
    case DEV_REMOTE_CONTROL_TASK_REQUEST: {
        RLM_REMOTE_CTRL_TASK_S *ctrl=(RLM_REMOTE_CTRL_TASK_S*)param;  /* 强转成遥控结构体 */

        TAL_PR_NOTICE("RLM remote ctrl, linear_x: %f angular_z: %f", ctrl->linear_x, ctrl->angular_z);  /* 打印方向参数 */

        if ( ctrl->linear_x>0.1f ) {
            send_ctrl_cmd_to_theone('w');   /* 前进：线速度为正 */
        } else if ( ctrl->linear_x<-0.1f ) {
            send_ctrl_cmd_to_theone('s');   /* 后退：线速度为负 */
        } else if ( ctrl->angular_z>0.1f ) {
            send_ctrl_cmd_to_theone('a');   /* 左转：角速度为正 */
        } else if ( ctrl->angular_z<-0.1f ) {
            send_ctrl_cmd_to_theone('d');   /* 右转：角速度为负 */
        } else {
            send_ctrl_cmd_to_theone('g');   /* 停止：方向都接近 0 */
        }
        break;   /* 遥控任务没有 response 接口 */
    }
    case IMMEDIATE_MOWING_TASK_REQUEST: {  /* 割草任务命令 */
        RLM_IMMEDIATE_MOWING_TASK_S *task=(RLM_IMMEDIATE_MOWING_TASK_S*)param;  /* param 转成割草任务结构体 */
        RLM_IMMEDIATE_MOWING_TASK_R task_res={0};  /* 应答结构体，先清零 */

        TAL_PR_NOTICE("RLM mowing task, go: %d mode: %d area_num: %d", task->ctrl_switch_go, task->mow_mode, task->area_num);  /* 打印云端下发的割草参数 */
        if ( task->ctrl_switch_go ) {  /* go=1 表示开始割草 */
            send_ctrl_cmd_to_theone('3');  /* 开始割草 */
        } else {
            send_ctrl_cmd_to_theone('0');  /* 停止 */
        }

        task_res.ctrl_switch_go=task->ctrl_switch_go;  /* 应答里回填开关值 */
        task_res.mow_mode=task->mow_mode;  /* 回填割草模式 */
        task_res.area_num=task->area_num;  /* 回填区域数量 */
        task_res.area_id=task->area_id;  /* 回填区域 ID */
        task_res.code_error.success=TRUE;  /* 标记执行成功 */
        ret=ty_rlm_immediate_mowing_task_response(&task_res);  /* 调应答接口把结果上报云端 */
        break;
    }
    case IMMEDIATE_CHARGE_TASK_REQUEST: {  /* 回充任务命令 */
        RLM_IMMEDIATE_CHARGE_TASK_S *charge=(RLM_IMMEDIATE_CHARGE_TASK_S*)param;  /* param 转成回充任务结构体 */
        RLM_IMMEDIATE_CHARGE_TASK_R charge_res={0};  /* 应答结构体，先清零 */

        TAL_PR_NOTICE("RLM charge task, charge: %d", charge->ctrl_switch_charge);  /* 打印下发的回充开关 */
        if ( charge->ctrl_switch_charge ) {  /* charge=1 表示开始回充 */
            send_ctrl_cmd_to_theone('h');  /* 开始回充 */
        } else {
            send_ctrl_cmd_to_theone('0');  /* 停止，具体指令待确认 */
        }

        charge_res.ctrl_switch_charge=charge->ctrl_switch_charge;  /* 回填开关值 */
        charge_res.code_error.success=TRUE;  /* 标记执行成功 */
        ret=ty_rlm_immediate_charge_task_response(&charge_res);  /* 调应答接口把结果上报云端 */
        break;
    }
    default:
        TAL_PR_NOTICE("rlm set, unsupported cmd: %d", cmd);  /* 暂不支持的 RLM 命令 */
        break;
    }
    return ret;
}

/**
 * @brief RLM 查询命令回调，App 查询时由 SDK 调用
 */
static OPERATE_RET rlm_advance_function_query(IN COMPLEX_RLM_CMD_E cmd, IN void* param)  /* RLM QUERY 回调：App 查询时 SDK 调用 */
{
    TAL_PR_NOTICE("rlm query cmd: %d", cmd);  /* 打印查询命令 */
    /* TODO: 按命令查询本地数据并调用对应 response 接口上报 */
    return OPRT_OK;
}
/* ========== [新增代码结束] ========== */

// /* ========== [临时测试] 模拟云端下发 RLM 割草命令 ========== */
// static VOID_T test_rlm_mowing(VOID_T)
// {
//     RLM_IMMEDIATE_MOWING_TASK_S task={0};

//     task.ctrl_switch_go=TRUE;
//     task.mow_mode=0;
//     task.area_num=0;
//     rlm_advance_function_set(IMMEDIATE_MOWING_TASK_REQUEST, &task);
// }
/* ========== [临时测试结束] ========== */

/* ============================================================
 *  OTA 重启标志处理
 * ============================================================ */
#define OTA_UPDATE_FLAG_FILE   "/userdata/ota_update_pending.flag"

/**
 * @brief 启动时检查 OTA 升级标志
 *
 * 本地升级程序在重启前可写入 OTA_UPDATE_FLAG_FILE；
 * 设备重启后首次运行到这里，说明升级流程已走完，打印确认并清理标志。
 */
static VOID_T __ota_clear_update_flag_on_boot(VOID_T)
{
    if (access(OTA_UPDATE_FLAG_FILE, F_OK) == 0) {
        TAL_PR_NOTICE("=== OTA reboot detected, upgrade applied ===");
        if (unlink(OTA_UPDATE_FLAG_FILE) != 0) {
            TAL_PR_WARN("ota clear flag failed: %s", strerror(errno));
        }
    }
}

/* ============================================================
 *  本地授权信息写入（调试用）
 * ============================================================ */
static OPERATE_RET __set_auth_info(VOID_T)
{
    OPERATE_RET rt = OPRT_OK;
    GW_BASE_IF_S bi = {0};

    /* 若已存在授权信息则不重复覆盖，避免调试时频繁写入 */
    rt = wd_gw_base_if_read(&bi);
    TAL_PR_NOTICE("wd_gw_base_if_read ret=%d, uuid[%s], auth_key[%s]",
                  rt, bi.uuid, bi.auth_key);
    if (rt == OPRT_OK && strlen(bi.uuid) > 0 && strlen(bi.auth_key) > 0) {
        TAL_PR_NOTICE("auth info already exist, skip write");
        return OPRT_OK;
    }

    memset(&bi, 0, sizeof(bi));
    snprintf(bi.uuid, sizeof(bi.uuid), "%s", USER_TUYA_UUID);
    snprintf(bi.auth_key, sizeof(bi.auth_key), "%s", USER_TUYA_AUTHKEY);
    snprintf(bi.psk_key, sizeof(bi.psk_key), "%s", "");
    snprintf(bi.psk30_key, sizeof(bi.psk30_key), "%s", "");
    snprintf(bi.fac_pin, sizeof(bi.fac_pin), "%s", USER_TUYA_PID);
    bi.prod_test = FALSE;

    rt = wd_gw_base_if_write(&bi);
    if (rt != OPRT_OK) {
        TAL_PR_ERR("wd_gw_base_if_write failed: %d", rt);
        return rt;
    }
    TAL_PR_NOTICE("set auth info ok");

    return rt;

    // return OPRT_OK;   /* 由 App 配网自动授权 */
}

/* ========== [新增代码] IPC 本地存储适配桩（SD 卡） ========== */
/* ============================================================
 *  IPC 本地存储适配接口（SD 卡相关）
 *  本项目不使用 IPC 本地存储，但 libtuyaos.a 的 IPC 组件在链接
 *  时需要应用层提供以下接口，否则报 undefined reference。
 *  这里统一按"无 SD 卡、不写存储"实现。
 * ============================================================ */
SD_STATUS_E tuya_ipc_sd_get_status(VOID)  /* 返回“没有 SD 卡” */
{
    return SD_STATUS_NOT_EXIST;  /* 枚举值：SD 卡不存在 */
}

VOID tuya_ipc_sd_get_capacity(UINT_T *p_total, UINT_T *p_used, UINT_T *p_free)  /* 返回 SD 卡容量，这里全部填 0 */
{
    if ( p_total ) {  /* 指针判空（代码规范：判存在用 ptr） */
        *p_total=0;  /* 总容量 0 */
    }
    if ( p_used ) {  /* 指针判空 */
        *p_used=0;  /* 已用容量 0 */
    }
    if ( p_free ) {  /* 指针判空 */
        *p_free=0;  /* 剩余容量 0 */
    }
}

VOID tuya_ipc_sd_remount(VOID)  /* 重新挂载 SD 卡，本项目不处理 */
{
}

CHAR_T *tuya_ipc_get_sd_mount_path(VOID)  /* 返回 SD 卡挂载路径 */
{
    return (CHAR_T *)"/tmp";  /* 给一个可写目录占位 */
}

STREAM_STORAGE_WRITE_MODE_E tuya_ipc_sd_get_mode_config(VOID)  /* 返回存储写模式 */
{
    return SS_WRITE_MODE_NONE;  /* 不写录像 */
}
/* ========== [新增代码结束] ========== */

/* ============================================================
 *  主入口
 * ============================================================ */
int main(int argc, char *argv[])
{
    OPERATE_RET rt = OPRT_OK;

    /* 1. 设置日志等级 */
    tuya_iot_set_log_attr(TAL_LOG_LEVEL_NOTICE);

    TAL_PR_NOTICE("================================");
    TAL_PR_NOTICE("X3M BLE+WiFi Netcfg Demo Start");
    TAL_PR_NOTICE("================================");

    /* 2. 创建存储目录（SDK KV/数据库需要）。
     *    /userdata 被清空后，必须逐级创建父目录。 */
    mkdir("/userdata", 0755);
    mkdir(TY_APP_STORAGE_PATH, 0755);
    mkdir(TY_SDK_ONLINE_LOG_PATH, 0755);
    mkdir("/userdata/app", 0755);
    mkdir("/userdata/app/Version", 0755);

    /* 2.1 如果默认 OTA 目录 /app/TheOne/Ver 无法写入，降级到 /tmp。
     *     涂鸦不要求必须挂在 /userdata，任何持久化目录都可以。 */
    mkdir("/app", 0755);
    mkdir("/app/TheOne", 0755);
    mkdir("/app/TheOne/Ver", 0755);
    if (access("/app/TheOne/Ver", F_OK) != 0 || access("/app/TheOne/Ver", W_OK) != 0) {
        TAL_PR_WARN("/app/TheOne/Ver not writable, use /tmp/tuya_x3m for ota storage");
        snprintf(g_ota_tmp_path, sizeof(g_ota_tmp_path), "%s", OTA_PACKAGE_TMP_PATH_FALLBACK);
        snprintf(g_ota_final_path, sizeof(g_ota_final_path), "%s", OTA_PACKAGE_FINAL_PATH_FALLBACK);
    }

    /* 3. 开启蓝牙服务并设置 BLE 配网能力（在 tuya_iot_init_params 之前调用）。
     *    SDK 在系统服务初始化时会根据这些标志自动初始化并打开 BLE 服务。 */
    tuya_ble_set_serv_switch(TRUE);
    tuya_ble_set_startup_attr(TUYA_BLE_ABILITY_NETCFG);
    tuya_enable_ble_netcfg(TRUE);
    TAL_PR_NOTICE("ble netcfg ability enabled");

    /* 4. 系统服务初始化。 */
    TY_INIT_PARAMS_S init_param = {0};
    init_param.init_db = TRUE;
    snprintf(init_param.sys_env, SYS_ENV_LEN, "%s", TARGET_PLATFORM);
    snprintf(init_param.log_seq_path, LOG_SEQ_PATH_LEN, "%s", TY_SDK_ONLINE_LOG_PATH);

    rt = tuya_iot_init_params(TY_APP_STORAGE_PATH, &init_param);
    if (rt != OPRT_OK) {
        TAL_PR_ERR("tuya_iot_init_params failed: %d", rt);
        return -1;
    }
    TAL_PR_NOTICE("tuya iot init ok");

    /* 4.1 启动时检查 OTA 升级标志，确认是否从升级重启中恢复 */
    __ota_clear_update_flag_on_boot();

    /* 5. 写入授权信息（仅调试用，量产请使用涂鸦授权工具） */
    rt = __set_auth_info();
    if (rt != OPRT_OK) {
        TAL_PR_ERR("set auth info failed, continue anyway");
    }

    /* 6. 不手动调用 tuya_svc_bt_init()，由 tuya_iot_wf_soc_dev_init_param 自动打开 BLE 服务。 */

    /* 7. 注册网络状态回调 */
    tuya_iot_reg_get_wf_nw_stat_cb(__wf_nw_stat_cb);
    tuya_iot_set_wf_cfg_err_code_cb(__wf_cfg_err_code_cb);

    /* 8. 初始化回调结构体，供 tuya_iot_wf_soc_dev_init_param 使用 */
    TY_IOT_CBS_S iot_cbs = {
        .gw_status_cb  = __gw_status_changed_cb,
        .gw_ug_cb      = __gw_ug_cb,
        .gw_reset_cb   = __gw_reset_cb,
        .dev_obj_dp_cb = __dev_obj_dp_cb,
        .dev_raw_dp_cb = __dev_raw_dp_cb,
        .dev_dp_query_cb = __dev_dp_query_cb,
        .dev_ug_cb     = __dev_ug_cb,
        .dev_reset_cb  = NULL,
        .active_shorturl = NULL,
        .pre_gw_ug_cb  = __ota_pre_check,
        .pre_dev_ug_cb = __ota_pre_check_dev,
    };

    TAL_PR_NOTICE("[OTA-CB] iot_cbs registered: gw_ug_cb=%p, dev_ug_cb=%p, pre_gw_ug_cb=%p, pre_dev_ug_cb=%p",
                  iot_cbs.gw_ug_cb, iot_cbs.dev_ug_cb,
                  iot_cbs.pre_gw_ug_cb, iot_cbs.pre_dev_ug_cb);

    /* 9. 在 Wi-Fi SOC 初始化前，先把网关工作状态重置为未注册/未配网。
     *    这样即使 KV 里还残留上次激活的状态，SDK 也会重新进入配网流程，
     *    而不会直接关闭 BLE 配网服务。 */
    GW_WORK_STAT_MAG_S wsm = {0};
    OPERATE_RET wsm_rt = wd_gw_wsm_read(&wsm);
    if (wsm_rt != OPRT_OK) {
        TAL_PR_WARN("wd_gw_wsm_read failed: %d, use default uncfg wsm", wsm_rt);
    }
    wsm.nc_tp = GWNS_UNCFG_SMC;
    wsm.md = GWM_SPECIAL_SMT_CFG;
    wsm.stat = UNREGISTERED;
    memset(wsm.ssid, 0, sizeof(wsm.ssid));
    memset(wsm.passwd, 0, sizeof(wsm.passwd));
    wsm_rt = wd_gw_wsm_write(&wsm);
    TAL_PR_NOTICE("force wsm to unregistered uncfg, write ret: %d, stat=%d", wsm_rt, wsm.stat);

    /* 9.1 显式把 Wi-Fi 启动模式保存到 KV，并设置 Wi-Fi link 参数，
     *     禁止 station+ap 并发模式，避免 SDK 初始化时启动 AP。 */
    wifi_save_start_mode(WF_START_SMART_ONLY);
    TUYA_WIFI_CFG_PARAM wifi_cfg_param = {
        .mthd = GWCM_OLD,
        .start_mode = WF_START_SMART_ONLY,
        .enable_station_ap = FALSE,
    };
    set_wifi_config_params(wifi_cfg_param);
    TAL_PR_NOTICE("wifi cfg param set: mthd=GWCM_OLD, start_mode=SMART_ONLY, enable_station_ap=FALSE");

    rt = tuya_iot_wf_soc_dev_init_param(
            GWCM_OLD,                /* 上电配网模式：无低功耗，启动即进入待配网状态 */
            WF_START_SMART_ONLY,     /* 不用 AP，同时启用 BLE + Wi-Fi 快连 */
            &iot_cbs,
            USER_FIRMWARE_KEY,
            USER_TUYA_PID,
            USER_SW_VER);
    if (rt != OPRT_OK) {
        TAL_PR_ERR("tuya_iot_wf_soc_dev_init_param failed: %d", rt);
        return -1;
    }
    TAL_PR_NOTICE("wifi soc dev init ok");

/* ========== [新增代码] RLM 回调注册 ========== */
    /* 注册 RLM 复杂功能回调，处理云端下发的割草/回充等命令 */
    rt = ty_rlm_complex_func_register(rlm_advance_function_set, rlm_advance_function_query);  /* 把 SET/QUERY 回调注册给 SDK（必须在联网初始化之前） */
    if ( rt!=OPRT_OK ) {  /* 注册失败处理 */
        TAL_PR_ERR("ty_rlm_complex_func_register failed: %d", rt);  /* 打印失败错误码 */
    }

    /* 启动 TheOne TCP 客户端线程：发指令 + 收心跳上报 DP113 */
    theone_start();

    // /* ========== [临时测试] 模拟云端下发一次 ========== */
    // test_dp_downlink();
// /* ========== [新增代码结束] ========== */
//     sleep(5);   /* 等 theone 连接建立 */
//     test_rlm_mowing();

    /* 9.2 显式初始化 OTA 升级服务。... */

    /* 9.2 显式初始化 OTA 升级服务。
     *     TY_IOT_CBS_S 中的 pre_dev_ug_cb 签名带 dev_id，但 tuya_svc_upgrade
     *     实际调用的是 TUYA_UPGRADE_PARAM_T.pre_dev_upgrade_cb（不带 dev_id）。
     *     SOC 产品必须额外调用 tuya_svc_upgrade_init 注册该回调，否则云端
     *     推送升级后会报 "pre dev ug check failed, abort upgrade"。 */
    TUYA_UPGRADE_PARAM_T upgrade_param = {
        .dev_upgrade_cb        = __dev_upgrade_inform_cb,
        .subdev_upgrade_cb     = NULL,
        .pre_dev_upgrade_cb    = __pre_dev_upgrade_cb,
        .pre_subdev_upgrade_cb = NULL,
    };
    rt = tuya_svc_upgrade_init(&upgrade_param);
    if (rt != OPRT_OK) {
        TAL_PR_ERR("tuya_svc_upgrade_init failed: %d", rt);
        return -1;
    }
    TAL_PR_NOTICE("[OTA-CB] tuya_svc_upgrade_init ok");

    /* 10. 确认 BLE 服务已由 SDK 自动打开。
     *    0=TUYA_BT_SERV_OPENED, 1=OPENING, 2=CLOSED, 3=CLOSING */
    TAL_PR_NOTICE("ble serv stat after wf soc init: %d", tuya_ble_get_serv_stat());

    /* 11. tuya_iot_wf_soc_dev_init_param 内部已经自动调用了 tuya_iot_netcfg_init，
     *     应用层不需要再显式调用，否则会因为 netcfg 模块重复注册而返回 -2。
     *     我们只需要在前面把 WSM 写成合法未配网状态，SDK 会自动启动 BLE+Wi-Fi 配网。 */
    TAL_PR_NOTICE("netcfg auto inited by sdk, waiting for app...");
    TAL_PR_NOTICE("ble serv stat right after wf soc init: %d", tuya_ble_get_serv_stat());

    /* 12. 主循环 */
    int ble_stat_cnt = 0;
    int last_ble_stat = -1;
    while (1) {
        int cur_ble_stat = tuya_ble_get_serv_stat();
        if (cur_ble_stat != last_ble_stat || (ble_stat_cnt % 5 == 0)) {
            TAL_PR_NOTICE("[BLE-STAT-MON] ble serv stat: %d (0=OPENED,1=OPENING,2=CLOSED,3=CLOSING)", cur_ble_stat);
            last_ble_stat = cur_ble_stat;
        }
        ble_stat_cnt++;
        sleep(1);
    }

    return 0;
}




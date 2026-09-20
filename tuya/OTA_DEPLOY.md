# RDK X3 TuyaOS OTA 部署与运维指南（对接本地升级程序）

本文档说明如何在 RDK X3（aarch64 Linux）上，让 TuyaOS 只负责“下载并保存升级包”，然后触发 X3 本地已有的升级程序完成后续安装。

## 1. OTA 架构

```
涂鸦云平台
    │
    ▼
TuyaOS svc_upgrade（云端下载、HMAC/签名校验、版本管理）
    │
    ▼
TAL tal_ota（OTA 抽象层）
    │
    ▼
TKL tkl_ota.c（RDK X3 适配层：接收升级包 → 保存到 /userdata → 触发本地升级程序）
    │
    ▼
X3 本地升级程序（解压、校验、刷写、重启，由用户自行实现或已有）
```

## 2. 关键文件路径

| 文件/路径 | 说明 |
|-----------|------|
| `/userdata/app/Version/ota_package.zip.tmp` | 下载过程中临时缓存 |
| `/userdata/app/Version/ota_package.zip` | 下载完成后最终落盘路径 |
| `/userdata/ota_update.sh` | 示例本地升级脚本（可选，可设为空） |
| `/userdata/tuya_x3m/` | SDK KV 与日志持久化目录 |
| `/userdata/tuyaos_demo_x3m_ble_wifi_netcfg` | TuyaOS 应用本体 |
| 用户自己的开机自启脚本 | 由用户自行维护，不在本仓库中 |

## 3. 源码修改点

### 3.1 TKL 适配层：tkl_ota.c

实现 `tkl_ota_get_ability`、`tkl_ota_start_notify`、`tkl_ota_data_process`、`tkl_ota_end_notify`：

- 升级包缓存到 `/userdata`，避免 `/tmp` 掉电丢失。
- 下载阶段按 `pack->offset` 随机写入，兼容 SDK 重传/乱序分片。
- 结束阶段校验文件大小，确认包完整。
- 自动创建 `/userdata/app/Version` 目录。
- 将临时包 `rename` 到最终路径 `/userdata/app/Version/ota_package.zip`。
- 使用 `fork + /bin/sh -c` **后台启动**本地升级程序，不阻塞 TuyaOS（命令可设为空）。


**需要你修改的宏（文件顶部）：**

```c
#define TKL_OTA_PACKAGE_FINAL    "/userdata/app/Version/ota_package.zip"
#define TKL_OTA_UPGRADE_CMD      "/userdata/ota_update.sh %s"
```

- 若只想保存 zip 包、不触发任何程序：把 `TKL_OTA_UPGRADE_CMD` 设为空字符串 `""`。
- 若本地程序不需要参数：`"/userdata/ota_update.sh"`
- 若本地程序是二进制并带参数：`"/usr/bin/ota_update --package=%s"`

### 3.2 应用入口：main.c

- `pre_gw_ug_cb` 注册为 `__ota_pre_check`，在云端下发升级时优先检查 `/userdata` 磁盘空间。
- `gw_ug_cb` 打印升级信息并返回 `OPRT_OK` 接受升级。
- `TY_APP_STORAGE_PATH` 与 `TY_SDK_ONLINE_LOG_PATH` 设置为 `/userdata/tuya_x3m/`，确保 KV 数据库和激活状态在 OTA 重启后不丢失。

## 4. 涂鸦把包传完之后会做什么？

1. 云端下发升级通知 → 触发 `pre_gw_ug_cb` → 触发 `gw_ug_cb`。
2. 设备返回 `OPRT_OK` 后，SDK 开始从云端 HTTP 下载固件包。
3. 下载过程中反复调用 `tkl_ota_data_process()` 把分片写入 `/userdata/app/Version/ota_package.zip.tmp`。
4. 下载结束后，SDK 内部完成 HMAC/签名校验。
5. 校验通过，调用 `tkl_ota_end_notify(reset=TRUE)`。
6. **到此为止，涂鸦的工作就结束了** —— 它不会解压、不会替换分区、不会重启。
7. 剩下的安装动作由 `tkl_ota_end_notify()` 触发你的本地升级程序完成。

## 5. 本地升级程序示例

假设你的升级包是 zip，X3 上已有脚本 `/userdata/ota_update.sh`：

```bash
#!/bin/bash
set -e

PKG="$1"
WORK_DIR="/userdata/ota_work"

log() {
    echo "[LOCAL-OTA] $1"
}

if [ -z "$PKG" ] || [ ! -f "$PKG" ]; then
    log "package not found: $PKG"
    exit 1
fi

log "unpack $PKG ..."
mkdir -p "$WORK_DIR"
rm -rf "$WORK_DIR/*"
unzip -o "$PKG" -d "$WORK_DIR"

log "run install script ..."
# 这里调用你 X3 上真正的安装脚本/命令
bash "$WORK_DIR/install.sh"

log "cleanup and reboot ..."
rm -f "$PKG"
rm -rf "$WORK_DIR"
sync
reboot
```

然后在 `tkl_ota.c` 中配置（已默认）：

```c
#define TKL_OTA_PACKAGE_FINAL    "/userdata/app/Version/ota_package.zip"
#define TKL_OTA_UPGRADE_CMD      "/userdata/ota_update.sh %s"
```

## 6. 编译

```bash
cd X3M_TuyaOS-3.9.1/software/TuyaOS
sh build_app.sh tuyaos_demo_x3m_ble_wifi_netcfg 1.0.0
```

产物：

```
./apps/tuyaos_demo_x3m_ble_wifi_netcfg/output/tuyaos_demo_x3m_ble_wifi_netcfg_/tuyaos_demo_x3m_ble_wifi_netcfg
```

## 7. 部署到 RDK X3

```bash
# TuyaOS 应用
scp ./apps/tuyaos_demo_x3m_ble_wifi_netcfg/output/tuyaos_demo_x3m_ble_wifi_netcfg_/tuyaos_demo_x3m_ble_wifi_netcfg root@<ip>:/userdata/

# 你的本地升级程序（可选，若 TKL_OTA_UPGRADE_CMD 为空可跳过）
scp /path/to/your/ota_update.sh root@<ip>:/userdata/

ssh root@<ip> "chmod +x /userdata/ota_update.sh 2>/dev/null; /userdata/tuyaos_demo_x3m_ble_wifi_netcfg"
```

> 开机自启请使用你自己的启动脚本，本仓库不再提供 systemd service 文件。

## 8. 触发云端 OTA

1. 涂鸦开发者平台上传升级包（版本号高于当前 `USER_SW_VER`）。
2. 平台推送升级通知到设备。
3. 设备日志顺序：
   - `=== OTA Upgrade Inform ===`
   - `ota disk check: need=..., free=...`
   - `ota recv progress: ...`
   - `ota package ready: /userdata/app/Version/ota_package.zip`
   - `ota local upgrade program started, pid=..., cmd=...`（若命令非空）
4. 本地升级程序接管，完成解压、刷写、重启。

## 9. 常见问题

### 9.1 升级后设备需要重新配网

- 检查 `TY_APP_STORAGE_PATH` 是否为 `/userdata/tuya_x3m/`，不能放 `/tmp`。

### 9.2 下载阶段失败

- 检查 `/userdata` 可用空间是否大于升级包大小 + 10%。
- 查看 `ota recv progress` 确认分片写入正常。

### 9.3 本地升级程序没被执行

- 确认 `TKL_OTA_UPGRADE_CMD` 路径正确，且文件有可执行权限。
- 在 X3 上手动运行一次命令验证：`/bin/sh -c "/userdata/ota_update.sh /userdata/app/Version/ota_package.zip"`。
- 查看 TuyaOS 日志中 `ota local upgrade program started` 是否打印。

### 9.4 包被改名后找不到

- `tkl_ota_end_notify` 会把 `ota_package.zip.tmp` `rename` 成 `/userdata/app/Version/ota_package.zip`。
- 本地程序读取的参数路径就是最终路径，不要再去读 `.tmp`。

## 10. 安全与生产建议

- 量产时移除 `main.c` 中硬编码的 `DEMO_UUID` / `DEMO_AUTH_KEY`。
- 本地升级程序中建议做：签名/哈希校验、备份当前固件、失败回退、安全重启。
- 在 `__ota_pre_check()` 中加入业务保护：机器人运动/加热/充电时拒绝升级。

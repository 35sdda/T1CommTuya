# the1Comm（涂鸦通道版）

涂鸦通道接管 `the1Comm` 位置的测试版本。它**就是** the1 眼中的 comm 子进程，
在 `/app/TheOne/` 下替换原 `the1Comm` 做实测。

> **目录约定：平铺。** 所有 `.cpp` / `.hpp` / `.md` / `Makefile` / `local.mk` 都在同一层，
> 不分子目录。

## 和原 the1Comm 的关系

| 保留 | 删掉（测试期临时失去） |
|---|---|
| `T1IPCCommMain`（`/theone_m2c` `/theone_c2m`） | `T1BlueT`（蓝牙 SPP 遥控） |
| `T1IPCComm2Act`（`/theone_c2a` `/theone_a2c`） | `T1Socket`（1111，App 的 WiFi 本地 TCP） |
| 新增 `T1Tuya`（涂鸦云 DP / OTA） | `T1Mqtt`（EMQX `x3m/upgrade/url`、`x3m/device/status`） |

⚠️ 需求文档（`APP功能需求文档.md` :29 / :44 / :462）明确要求手动控制与视觉建图走
**本地 BLE + WiFi 本地 TCP，禁止上云**。所以这只是**测试期的临时替换**，
最终形态必须是「原 the1Comm 保留 1111 + 蓝牙，再把 T1Tuya 加进来」。

## 第 0 步（VM 上第一次编译前做一次）：修工具链权限 + 补符号链接

**必做，否则第一次编译一定失败。** SDK 从 Windows 拷过来时 POSIX 元数据全丢了，
而且丢的是**两样**东西：**可执行位**和**符号链接**。这是两个独立的坑，都要补。

### 0.1 补可执行位 —— 症状 `权限不够` / `Permission denied`

```
/bin/sh:行1: .../toolchain/.../bin/aarch64-none-linux-gnu-gcc: 权限不够
make: *** [.../tkl_init_pm.c.o] 错误 1
```

原因：`bin/` 下所有 `gcc / g++ / as / ld / ar / objcopy`，以及
`libexec/gcc/aarch64-none-linux-gnu/11.2.1/` 下的 `cc1` `cc1plus` `collect2` `lto1` `lto-wrapper`，
**全部是 `-rw-r--r--`，没有一个 `x` 位**。

```bash
cd ~/T1tuya/software/TuyaOS        # 换成你实际的 SDK 路径
TC=vendor/aarch64-none-linux-gnu-gcc-11.2.1/toolchain/gcc-arm-11.2-2022.02-x86_64-aarch64-none-linux-gnu

chmod -R +x $TC/bin
chmod -R +x $TC/libexec
chmod -R +x $TC/aarch64-none-linux-gnu/bin 2>/dev/null   # 该目录可能不存在，忽略报错
find . -name "*.sh" -exec chmod +x {} \;                 # SDK 有些脚本会被直接执行
```

验证：`ls -l $TC/bin/aarch64-none-linux-gnu-gcc` 应看到 `-rwxr-xr-x`。

### 0.2 补符号链接 —— 症状 `cannot find -lxxx`

Windows 文件系统没有符号链接概念，工具链里所有 `libxxx.so -> libxxx.so.x.y.z` 的链接名都丢了，
只剩下带版本号的真身。链接器找的是不带版本号的名字，于是
`cannot find -ldbus-1 / -lbluetooth / -lglib-2.0 / -lz / -lstdc++ ...`。

```bash
# 1) sysroot 里 22 个蓝牙/dbus/glib 等库，补出不带版本号的链接名
cd $TC/aarch64-none-linux-gnu/libc/usr/lib/aarch64-linux-gnu
for f in lib*.so.*; do ln -sf "$f" "${f%%.so.*}.so"; done
cd -

# 2) libstdc++：本工程有 .cpp，最终链接要 -lstdc++，而这个链接名也不存在
cd $TC/aarch64-none-linux-gnu/lib64
ln -sf libstdc++.so.6.0.29 libstdc++.so
cd -
```

验证（4 条都要有输出，指向带版本号的文件）：

```bash
ls -l $TC/aarch64-none-linux-gnu/libc/usr/lib/aarch64-linux-gnu/libdbus-1.so \
      $TC/aarch64-none-linux-gnu/libc/usr/lib/aarch64-linux-gnu/libbluetooth.so \
      $TC/aarch64-none-linux-gnu/libc/usr/lib/aarch64-linux-gnu/libz.so \
      $TC/aarch64-none-linux-gnu/lib64/libstdc++.so
```

## 编译

必须在 **x86_64 Linux**（虚拟机）上交叉编译，板子和 Windows 都编不了。
本工程已经在 SDK 的 `apps/the1Comm` 里了，直接编：

```bash
cd ~/T1tuya/software/TuyaOS          # 换成你实际的 SDK 路径
sh build_app.sh apps/the1Comm the1Comm 1.0.0
```

看到 `Build APP Finish` 即成功。产物：

```bash
ls -l apps/the1Comm/output/the1Comm_1.0.0/the1Comm
file  apps/the1Comm/output/the1Comm_1.0.0/the1Comm
# 期望：ELF 64-bit LSB executable, ARM aarch64, dynamically linked
```

以后改了 `apps/the1Comm/` 里的代码，**只要重跑那一条 `sh build_app.sh`** 就行 ——
每次都会 `make app_static_clean` 全量重编，几分钟起步，属正常。

**版本号第二位的奇偶决定 debug 编译**（`build_app.sh:24-27`）：偶数 → `APP_DEBUG=1`。
`1.0.0` 开调试，`1.1.0` 关。

### 编译报错对照

| 报错 | 原因 | 处理 |
|---|---|---|
| `bin/aarch64-none-linux-gnu-gcc: 权限不够` / `Permission denied` | 工具链可执行位被 Windows 拷贝丢了 | 做上面的「第 0 步」0.1 |
| `cannot find -ldbus-1 / -lbluetooth / -lglib-2.0 / -lz ...` | 工具链里的 `.so` 符号链接被 Windows 拷贝弄丢了 | 做上面的「第 0 步」0.2 |
| `cannot find -lstdc++` | `libstdc++.so` 这个链接名不存在（只有 `.so.6.0.29`） | 做上面的「第 0 步」0.2 第 2 条 |
| `undefined reference to std::__cxx11::...` | `-lstdc++` 没生效 | 检查 `Makefile` 最后一行 `LINKFLAGS += -lstdc++` 在不在，且在 `-Wl,--end-group` 之后 |
| `undefined reference to tal_log_print` 等 | SDK 的 `.a` 是 8 字节空占位 | 找发 SDK 的人要完整包（`libs/libtuyaapp_*.a` 是空的，只有 `libtuyaos.a` / `libtuyaos_adapter.a` 是真的） |
| `Makefile:XX: *** missing separator` | Makefile 被存成 CRLF 了（`\` 续行会断） | VSCode 里设 `files.eol: "\n"`，或 `sed -i 's/\r$//' Makefile` |
| 大量 warning、无 error | SDK 自带代码的警告 | 忽略 |

## 部署（在板子上替换 the1Comm）

### 1. 备份（必做）

```bash
sudo cp -a /app/TheOne/the1Comm /app/TheOne/the1Comm.bak
ls -l /app/TheOne/the1Comm*
```

### 2. 确认启动方式与 cwd

```bash
cat /home/sunrise/TheOne.sh     # 谁启动 the1
ls -l /app/TheOne/the1Update
```

`the1` 用 `execl("../the1Comm", ...)` 拉起子进程，`../` 相对的是 the1 的 cwd，
必须是 `/app/TheOne/T1one` —— 否则 `../Ver` 和 `../the1Update` 都会找错地方。

### 3. 换上去

```bash
sudo cp <新二进制> /app/TheOne/the1Comm
sudo chmod +x /app/TheOne/the1Comm
```

换之前先把整条进程树停掉（`the1` / `the1Act` / `the1Vision` / 老 `the1Comm`），
否则 mqueue 状态是旧的。

### 4. 重启并判定

```bash
sudo /home/sunrise/TheOne.sh
# 另开终端看日志
tail -f /app/TheOne/Log/*.log | grep -E "t1comm sub-process ready|T1CommMain|T1Tuya"
```

- **成功**：出现 `T1IPCMain2Comm-WaitForReady: t1comm sub-process ready.`
- **失败**：the1 会自己 `execv("../the1Update","--rollback")`，板子重启后回到上一版

> 本程序**故意**先发 `MSG_COMM_READY` 再起涂鸦 SDK，
> 因为 the1 的 `WaitForReady(5000)` 只有 5 秒，超时就直接全系统回滚。

### 5. 回滚

```bash
sudo cp -a /app/TheOne/the1Comm.bak /app/TheOne/the1Comm
sudo rm /app/TheOne/the1Comm.bak
# 重启 the1
```

## 实测两条链路

### 云 → 设备（这条能完整验证）

涂鸦 App 点 DP 103（雨天割草），期望日志：

```
T1Tuya-OnObjDp: cnt=1
T1Tuya-OnObjDp: dpid=103 type=0 value=1
T1DpMap-OnObjDp: dpid=103(雨天割草) kind=0 value=1
T1Tuya-RecvData: [cloud->device] $X:RAIN_MOW,1
T1DevOut-SendToDevice: [cloud->device] -> Act: $X:RAIN_MOW,1
```

最后一条说明帧已经写进 `/theone_c2a`，the1Act 那边应该能看到。

注：DP 103~107、109、110 目前只打日志不下发指令 —— 映射规则要以
`APP功能需求文档.md` 为准，猜错了会直接改变割草机行为。已实现的三条是文档写明动作的：
101 下发 0 重置寿命、102 下发 0 重置次数、108 下发 true 开 BLE。

### 设备 → 云（目前只能看到"收到但不认识"）

```
T1Tuya-SendData: unhandled frame: $X:...
```

这不是 bug —— the1Act 的状态帧格式与 DP 的映射规则还没对齐。
对齐后在这里翻译成 `T1Tuya_ReportDpValue(DP_ID_DEVICE_STATUS, n)` 之类的上报。

想手动验证"设备 → 云"通不通，用调试口：

```bash
# the1 已在跑（队列已建好）的前提下
sudo T1COMM_DEBUG_STDIN=1 /app/TheOne/the1Comm
# 然后敲： $D:113,1,3      回车
# 涂鸦 App 上 DP113 设备状态应变成 3(作业)
```

调试格式：`$D:<dpid>,<type>,<value>`   type: `0`=bool `1`=value

## 文件职责（合并时的去向）

| 文件 | 职责 | 将来合进真正 the1Comm 时 |
|---|---|---|
| `T1CommMain.cpp` | 事件循环 + 启动顺序 | 丢弃（用原 the1Comm 的 `T1CommMain.cpp` + 改 2 处） |
| `T1IPCCommMain.*` | the1 的 4 条 mqueue | 不需要（原仓库已有） |
| `T1IPCComm2Act.*` | the1Act 握手 | 不需要（原仓库已有） |
| `T1Tuya.*` | 涂鸦 SDK 调用、DP 回调、OTA | **原样拷** |
| `T1DpMap.*` | DP 表 + 下发分发 | **原样拷** |
| `T1DevOut.*` | 设备侧出口 | **原样拷**（头文件用 `void*` 就是为了两边一致） |

合并时 `T1CommMain.cpp` 只需要改两处：

1. `tuya.Init();` 之前插一行 `T1DevOut_SetSinks(&comm, &comm2act);`
2. 把 `tuya.Init()` 挪到 `comm.SendToMain(MSG_COMM_READY, nullptr);` **之后**

## 授权信息

`T1Tuya.cpp` 顶部写死了调试用的 PID / UUID / AuthKey（从原 demo 的 `main.c` 搬来）。
量产要改成授权工具写入，或从 `/app/TheOne/Ver/Version` 运行时读业务版本号。

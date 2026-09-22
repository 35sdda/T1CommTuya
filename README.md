# the1Comm（涂鸦通道）编译步骤

## 0. 目录约定

平铺：所有 `.cpp` / `.hpp` / `.md` / `Makefile` / `local.mk` 都在同一层，没有 `src/`、`include/` 子目录。

## 1. 首次编译前：修工具链权限与符号链接（只做一次）

SDK 从 Windows 拷到 Linux 会丢可执行位和符号链接，不修第一次编译一定失败。

```bash
cd ~/T1tuya/software/TuyaOS        # 换成实际的 SDK 路径
TC=vendor/aarch64-none-linux-gnu-gcc-11.2.1/toolchain/gcc-arm-11.2-2022.02-x86_64-aarch64-none-linux-gnu

chmod -R +x $TC/bin
chmod -R +x $TC/libexec
chmod -R +x $TC/aarch64-none-linux-gnu/bin 2>/dev/null   # 该目录可能不存在，忽略报错
find . -name "*.sh" -exec chmod +x {} \;
```

验证：`ls -l $TC/bin/aarch64-none-linux-gnu-gcc` 应显示 `-rwxr-xr-x`（有 `x` 位）。

## 2. 编译

```bash
cd ~/T1tuya/software/TuyaOS
sh build_app.sh apps/the1Comm the1Comm 1.0.0
```

以后只改了 `T1Tuya.cpp` 之类的源码，重跑这一条命令就行。

## 3. 产物

```
apps/the1Comm/output/the1Comm_1.0.0/the1Comm
```

# Jetson Humble：CH341（ALX）驱动持久化安装

本说明适用于以下已确认场景：

- 背包为 Jetson，运行 ROS 2 Humble；
- `lsusb` 能看到 `1a86:7523 QinHeng Electronics CH340 serial converter`；
- 系统内核为 `5.15.148-tegra`，且未启用内置 CH341 驱动；
- ALX 接收器需要作为 `/dev/ttyUSB0` 使用。

本流程只会向当前内核的模块目录安装一个额外的 `ch341.ko`，不会修改 ROS 2、FAST-LIO 或任何 colcon workspace。

> 执行前请停止可能正在访问 ALX 或驱动机器人的程序。首次验证和重启验证均不会启动机器人运动控制。

## 1. 确认问题与编译条件

```bash
KVER="$(uname -r)"

lsusb
ls -l /dev/ttyUSB* 2>/dev/null
find "/lib/modules/$KVER/kernel/drivers/usb/serial" \
  -maxdepth 1 -type f -printf '%f\n' 2>/dev/null | sort

sudo modprobe usbserial
lsmod | grep usbserial

KBUILD="$(readlink -f "/lib/modules/$KVER/build")"
printf 'kernel=%s\nbuild=%s\n' "$KVER" "$KBUILD"
test -f "$KBUILD/Makefile" && echo '内核构建目录可用'
```

对于本背包，预期看到：

```text
5.15.148-tegra
usbserial.ko
内核构建目录可用
```

如果 `usbserial.ko` 不存在或 `Makefile` 缺失，请先安装与 `uname -r` **完全匹配**的 NVIDIA L4T kernel headers；不要安装通用 Ubuntu 内核模块，也不要从另一台机器复制 `.ko` 文件。

## 2. 构建与临时验证

下载 Linux 5.15 的 CH341 驱动源码，并针对正在运行的内核构建模块：

```bash
KVER="$(uname -r)"
KBUILD="$(readlink -f "/lib/modules/$KVER/build")"
WORK="$HOME/ch341-$KVER"

mkdir -p "$WORK"
cd "$WORK"

curl -4 --connect-timeout 15 --max-time 120 \
  --retry 2 --retry-all-errors -fL \
  'https://raw.githubusercontent.com/torvalds/linux/v5.15/drivers/usb/serial/ch341.c' \
  -o ch341.c

printf '%s\n' 'obj-m += ch341.o' > Makefile
make -C "$KBUILD" M="$PWD" modules

modinfo "$WORK/ch341.ko" | sed -n '1,16p'
```

通过条件：

- 已生成 `$WORK/ch341.ko`；
- `depends:` 包含 `usbserial`；
- `vermagic:` 与 `uname -r` 一致，例如 `5.15.148-tegra ... aarch64`。

然后临时加载并确认 ALX 枚举为串口：

```bash
sudo insmod "$WORK/ch341.ko"
sleep 1

lsmod | grep -E 'ch341|usbserial'
ls -l /dev/ttyUSB* 2>/dev/null
sudo dmesg | tail -n 30
```

成功时应出现：

```text
ch341 ...
usbserial ... ch341
/dev/ttyUSB0
ch341-uart converter now attached to ttyUSB0
```

如果下载不到源码，可在能访问该 URL 的开发电脑下载 `ch341.c`，再复制到 `$WORK/ch341.c` 后继续执行 `make`。如果 `insmod` 报 `Required key not available`，表示系统强制模块签名；不要绕过安全启动设置，应为该模块配置受信任签名或使用包含 CH341 的内核。

## 3. 安装为开机自动加载模块

仅在第 2 节临时验证成功后执行：

```bash
KVER="$(uname -r)"
WORK="$HOME/ch341-$KVER"

test -r "$WORK/ch341.ko" || {
  echo "未找到 $WORK/ch341.ko；请先完成第 2 节。"
  exit 1
}

sudo install -D -m 0644 \
  "$WORK/ch341.ko" \
  "/lib/modules/$KVER/extra/ch341.ko"

sudo depmod -a "$KVER"
printf '%s\n' ch341 | sudo tee /etc/modules-load.d/ch341.conf >/dev/null

modinfo -k "$KVER" ch341 | sed -n '1,12p'
```

这会完成两件事：

1. 将模块安装到仅属于当前内核的 `/lib/modules/$KVER/extra/ch341.ko`；
2. 让 systemd 在每次开机时加载 `ch341`。

重启前可检查系统是否已索引该模块：

```bash
modinfo -k "$(uname -r)" ch341 | grep -E '^(filename|name|vermagic):'
```

## 4. 重启后的验收

重启前，确认没有现场测试或机器人控制任务正在运行：

```bash
sudo reboot
```

重新登录后，ALX 保持接入状态，执行：

```bash
lsmod | grep -E 'ch341|usbserial'
ls -l /dev/ttyUSB0
sudo dmesg | grep -iE 'ch341|ttyUSB0' | tail -n 20
```

通过条件：

```text
/dev/ttyUSB0 存在
ch341 已加载
ch341-uart converter now attached to ttyUSB0
```

此时项目配置可保持：

```yaml
alx:
  port: /dev/ttyUSB0
  baudrate: 115200
```

## 5. 串口权限

设备通常属于 `root:dialout`：

```bash
ls -l /dev/ttyUSB0
id -nG
```

若输出中没有 `dialout`，将当前用户加入该组，然后退出当前 SSH/图形会话并重新登录：

```bash
sudo usermod -aG dialout "$USER"
```

不要用 `chmod 666 /dev/ttyUSB0` 作为长期方案；设备重插或重启后权限会恢复。

## 6. 内核升级后的处理

模块严格匹配内核版本。系统升级导致 `uname -r` 改变后，旧模块仍留在旧内核目录，但不会被新内核加载。此时：

1. 确认新内核对应的 NVIDIA L4T headers 已安装；
2. 从第 1 节开始重新执行；
3. 使用新的 `KVER` 重新构建并完成第 3 节安装；
4. 重启并按第 4 节验收。

可用以下命令快速发现内核变更：

```bash
uname -r
find /lib/modules -path '*/extra/ch341.ko' -print 2>/dev/null
```

## 7. 后续 ALX 数据验证

驱动和权限都通过后，再按 [docs/alx_ch340_usb_setup.md](docs/alx_ch340_usb_setup.md) 设置 `115200` 波特率并检查 ALX 原始帧；随后再启动 ROS 2 Humble / FAST-LIO / 跟随程序。

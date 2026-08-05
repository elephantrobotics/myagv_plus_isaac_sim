# myAGV Plus Isaac Sim 仿真环境搭建

分两步：先装 Isaac Sim 6.0.1（含驱动），再部署本仓库提供的 myAGV Plus 仿真案例（`humble_ws` + 离线场景包）。

---

## 一、安装 Isaac Sim 6.0.1

**系统要求**：Ubuntu 22.04 x86_64，NVIDIA 独显（RTX 系列），预留至少 50GB 磁盘空间。

### 1*. 检查 / 安装 NVIDIA 驱动

先检查机器上是不是已经有能用的驱动：

```bash
nvidia-smi
```

能正常显示显卡型号和驱动版本，**直接跳到「2. 下载并安装 Isaac Sim 6.0.1」，不用再装驱动**。

> ⚠️ **非必要不要装驱动**。Isaac Sim 对驱动具体版本不敏感，机器上现有的驱动只要能跑就够用，不是非得装成 595.58.03 这个版本不可。反而是**重复安装驱动**本身容易出问题——如果机器上同时存在系统自带/apt 装的驱动和另外手动装的驱动，会导致同一张 GPU 被注册两次（Vulkan ICD 冲突），装之前确认真的没有可用驱动再装。

如果 `nvidia-smi` 报错、确认没有可用驱动，再按下面手动装：

下载（百度网盘）：

- 通过网盘分享的文件：myAGV_Plus_Isaac_Sim资料下载
  链接: https://pan.baidu.com/s/1kBYK1mUhGqRrWju3glO2nQ?pwd=rdit 
  --来自百度网盘超级会员v9的分享
- 文件：`NVIDIA-Linux-x86_64-595.58.03.run`

```bash
chmod +x NVIDIA-Linux-x86_64-595.58.03.run
```

切到真实 TTY（`Ctrl+Alt+F3`）或从其他设备 SSH 进来操作——下一步会停掉图形界面，本地桌面会黑屏：

```bash
sudo systemctl isolate multi-user.target
sudo systemctl stop display-manager
sudo ./NVIDIA-Linux-x86_64-595.58.03.run
```

安装器选项：

| 提示 | 选择 |
|---|---|
| Kernel module type: Proprietary / Open / GPL | **Proprietary** |
| Install 32-bit compatibility libraries? | **No** |
| Register kernel module sources with DKMS? | **Yes** |
| Run `nvidia-xconfig`? | **No** |
| 提示 nouveau 冲突、要不要禁用并重建 initramfs | **Yes** |

```bash
sudo reboot
```

重启后验证：

```bash
nvidia-smi
```

能正常显示显卡型号和驱动版本即成功。

### 2. 下载并安装 Isaac Sim 6.0.1

前往官方下载页面，在 **Latest Release** 表格里点 **Linux (x86_64)**，下载 `isaac-sim-standalone-6.0.1-linux-x86_64.zip`：

<https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/download.html#isaac-sim-latest-release>

> 请从官方页面获取下载链接并确认许可协议——安装包本身不随本仓库分发。页面同时给出了 MD5 校验值，下载完可以顺手核对一下文件完整性。

```bash
mkdir ~/isaacsim
unzip "isaac-sim-standalone-6.0.1-linux-x86_64.zip" -d ~/isaacsim
cd ~/isaacsim
./post_install.sh
./isaac-sim.sh
```

首次启动会编译着色器缓存，耗时较久，属正常现象。

### 3. Compatibility Checker

```bash
cd ~/isaacsim
./isaac-sim.compatibility_check.sh
```

看到 `System checking result: PASSED` 再继续下一步。如果没通过，或 `nvidia-smi` 报错，见下方「常见问题」。

---

## 二、部署 myAGV Plus 仿真案例

**1. 获取 `humble_ws`（ROS2 工作空间源码）：**

仓库根目录下已经是 `humble_ws/` 子目录结构，克隆后把它复制到 `$HOME` 下：

```bash
cd ~
git clone https://github.com/elephantrobotics/myagv_plus_isaac_sim.git
mv myagv_plus_isaac_sim/humble_ws ~/humble_ws
rm -rf myagv_plus_isaac_sim
```

安装 ROS2 依赖：

```bash
sudo apt install -y ros-humble-geographic-msgs ros-humble-aruco-markers-msgs \
  ros-humble-robot-localization libgeographic-dev geographiclib-tools \
  ros-humble-geometry-msgs ros-humble-common-interfaces
```

编译：

```bash
cd ~/humble_ws
colcon build
echo "source ~/humble_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**2. 获取离线场景包：**

下载（百度网盘）：

- 通过网盘分享的文件：myAGV_Plus_Isaac_Sim资料下载
  链接: https://pan.baidu.com/s/1kBYK1mUhGqRrWju3glO2nQ?pwd=rdit 
  --来自百度网盘超级会员v9的分享
- 文件：`scenes.zip`

解压到 `~/isaacsim`：

```bash
unzip -o scenes.zip -d ~/isaacsim
```

解压后应得到 `~/isaacsim/scenes/` 目录，包含 4 套环境的 usda 与本地化资产。

---

## 常见问题

| 现象 | 原因 | 处理 |
|---|---|---|
| `nvidia-smi` 报 "couldn't communicate with the NVIDIA driver"，`dkms status` 能查到内核版本但对不上 `uname -r` | 内核自动升级后 DKMS 没针对新内核重编 | `sudo dkms autoinstall` 后 `sudo modprobe nvidia` |
| `nvidia-smi` 报错，且 `dkms status` 提示找不到命令 | 驱动和 DKMS 没装或被卸载 | `sudo apt install dkms linux-headers-$(uname -r)`，再重新执行上面「检查 / 安装 NVIDIA 驱动」的步骤 |
| 驱动装完 `nvidia-smi` 还是不通，`lspci -k` 显卡那行看不到 `Kernel driver in use: nvidia` | 可能是 Secure Boot 拦截了未签名的专有内核模块 | `mokutil --sb-state` 确认；若为 enabled，按驱动安装器提示走 MOK 签名注册（重启后蓝屏界面按提示操作），或去 BIOS 关闭 Secure Boot 后重装 |
| RTX Lidar / 渲染阶段报 `cudaErrorNoDevice` 等 CUDA 错误 | 驱动版本过旧或加载失败 | 先确认 `nvidia-smi` 正常，再跑 Compatibility Checker |
| Isaac 启动日志报 `[Error] [omni.rtx] Multiple Installable Client Drivers (ICDs) are found for the same GPU`；主界面画面正常，但 RTX 传感器（雷达等）没有数据 | 系统同时存在多套 NVIDIA 驱动（apt 装的 + 手动 `.run` 装的），GPU 被重复注册 | 见下方「多套驱动冲突」 |
| 打开场景报 `Failed to read texture ...amazonaws...` | 该场景仍引用云端资产，机器无法访问外网 | 使用本仓库提供的离线场景包（已本地化），不要用未 Collect 的开发版场景 |

**多套驱动冲突**：如果之前系统里已经装过驱动（比如用过 Ubuntu「软件更新」里的 Additional Drivers），装完本文档的 `.run` 驱动后可能会两套同时存在。检查：

```bash
ls /etc/vulkan/icd.d/ | grep nvidia
ls /usr/share/vulkan/icd.d/ | grep nvidia
dpkg -l | grep -i nvidia
```

如果两个目录都能找到 `nvidia_icd.json`，且 `dpkg -l` 里能看到 `nvidia-driver-xxx`、`libnvidia-*-xxx` 这类系统包，说明系统自带的驱动和手动装的 `.run` 驱动同时存在，需要卸掉系统自带那一套，只保留 `.run` 装的：

```bash
sudo apt purge '^nvidia-.*' '^libnvidia-.*' '^xserver-xorg-video-nvidia-.*' \
  '^linux-modules-nvidia-.*' '^linux-objects-nvidia-.*' '^linux-signatures-nvidia-.*' \
  '^screen-resolution-extra$'
sudo apt autoremove
sudo reboot
```

重启后再跑一次 `nvidia-smi` 确认驱动还正常。这一步不会动到 `.run` 装的驱动，它不受 apt 管理。

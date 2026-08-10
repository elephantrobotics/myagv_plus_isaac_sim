# myAGV Plus Isaac Sim Simulation Setup

[简体中文](README_CN.md)

Two steps: install Isaac Sim 6.0.1 (including the GPU driver), then deploy the myAGV Plus simulation package provided by this repository (`humble_ws` + the offline scene pack).

---

## 1. Install Isaac Sim 6.0.1

**System requirements**: Ubuntu 22.04 x86_64, NVIDIA discrete GPU (RTX series), at least 50 GB of free disk space.

### 1.1 Check / install the NVIDIA driver

First check whether a working driver is already installed:

```bash
nvidia-smi
```

If it prints your GPU model and driver version, **skip ahead to "1.2 Download and install Isaac Sim 6.0.1" — do not install a driver.**

> ⚠️ **Do not install a driver unless you have to.** Isaac Sim is not sensitive to the exact driver version; whatever already works on the machine is good enough, and 595.58.03 is not required. Installing a *second* driver is what causes trouble: if a distro/apt driver and a manually installed driver are both present, the same GPU gets registered twice (Vulkan ICD conflict). Confirm there is genuinely no working driver before installing one.

If `nvidia-smi` fails and you have confirmed there is no working driver, install it manually:

Download (Baidu Netdisk):

- Shared folder: myAGV_Plus_Isaac_Sim资料下载
  Link: https://pan.baidu.com/s/1kBYK1mUhGqRrWju3glO2nQ?pwd=rdit
- File: `NVIDIA-Linux-x86_64-595.58.03.run`

```bash
chmod +x NVIDIA-Linux-x86_64-595.58.03.run
```

Switch to a real TTY (`Ctrl+Alt+F3`) or connect over SSH from another machine — the next step stops the display manager, so the local desktop will go black:

```bash
sudo systemctl isolate multi-user.target
sudo systemctl stop display-manager
sudo ./NVIDIA-Linux-x86_64-595.58.03.run
```

Installer options:

| Prompt | Choose |
|---|---|
| Kernel module type: Proprietary / Open / GPL | **Proprietary** |
| Install 32-bit compatibility libraries? | **No** |
| Register kernel module sources with DKMS? | **Yes** |
| Run `nvidia-xconfig`? | **No** |
| Prompt about a nouveau conflict / rebuilding initramfs | **Yes** |

```bash
sudo reboot
```

Verify after rebooting:

```bash
nvidia-smi
```

Success means it prints your GPU model and driver version.

### 1.2 Download and install Isaac Sim 6.0.1

Open the official download page, find the **Latest Release** table, click **Linux (x86_64)**, and download `isaac-sim-standalone-6.0.1-linux-x86_64.zip`:

<https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/download.html#isaac-sim-latest-release>

> Get the download from the official page and accept the license there — the installer itself is not redistributed with this repository. The page also lists an MD5 checksum; verifying the downloaded file against it is worth the few seconds.

```bash
mkdir ~/isaacsim
unzip "isaac-sim-standalone-6.0.1-linux-x86_64.zip" -d ~/isaacsim
cd ~/isaacsim
./post_install.sh
./isaac-sim.sh
```

The first launch takes a while because the shader cache is being compiled. This is expected.

### 1.3 Compatibility Checker

```bash
cd ~/isaacsim
./isaac-sim.compatibility_check.sh
```

Wait for `System checking result: PASSED` before continuing. If it fails, or if `nvidia-smi` reports an error, see "Troubleshooting" below.

---

## 2. Deploy the myAGV Plus simulation package

**2.1 Get `humble_ws` (the ROS 2 workspace source):**

The repository already contains a `humble_ws/` subdirectory, so move it to `$HOME` after cloning:

```bash
cd ~
git clone https://github.com/elephantrobotics/myagv_plus_isaac_sim.git
mv myagv_plus_isaac_sim/humble_ws ~/humble_ws
rm -rf myagv_plus_isaac_sim
```

Install the ROS 2 dependencies:

```bash
sudo apt install -y ros-humble-geographic-msgs ros-humble-aruco-markers-msgs \
  ros-humble-robot-localization libgeographic-dev geographiclib-tools \
  ros-humble-geometry-msgs ros-humble-common-interfaces
```

Build:

```bash
cd ~/humble_ws
colcon build
echo "source ~/humble_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**2.2 Get the offline scene pack:**

Download (Baidu Netdisk):

- Shared folder: myAGV_Plus_Isaac_Sim资料下载
  Link: https://pan.baidu.com/s/1kBYK1mUhGqRrWju3glO2nQ?pwd=rdit
- File: `scenes.zip`

Extract into `~/isaacsim`:

```bash
unzip -o scenes.zip -d ~/isaacsim
```

You should end up with a `~/isaacsim/scenes/` directory containing the USDA files and localized assets for 4 environments.

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `nvidia-smi` reports "couldn't communicate with the NVIDIA driver", and `dkms status` lists kernel versions that do not match `uname -r` | The kernel was auto-upgraded and DKMS never rebuilt the module for it | `sudo dkms autoinstall`, then `sudo modprobe nvidia` |
| `nvidia-smi` fails and `dkms status` reports command not found | The driver and DKMS are missing or were removed | `sudo apt install dkms linux-headers-$(uname -r)`, then redo "1.1 Check / install the NVIDIA driver" above |
| Driver installed but `nvidia-smi` still fails, and `lspci -k` does not show `Kernel driver in use: nvidia` for the GPU | Secure Boot may be rejecting the unsigned proprietary kernel module | Check with `mokutil --sb-state`; if enabled, complete the MOK enrollment the installer offers (follow the blue screen after reboot), or disable Secure Boot in the BIOS and reinstall |
| CUDA errors such as `cudaErrorNoDevice` during RTX Lidar or rendering | Driver too old, or it failed to load | Confirm `nvidia-smi` works first, then run the Compatibility Checker |
| Isaac's startup log shows `[Error] [omni.rtx] Multiple Installable Client Drivers (ICDs) are found for the same GPU`; the main viewport renders fine but RTX sensors (lidar, etc.) produce no data | Multiple NVIDIA drivers are installed (apt plus a manual `.run`), so the GPU is registered twice | See "Conflicting drivers" below |
| Opening a scene reports `Failed to read texture ...amazonaws...` | That scene still references cloud assets and the machine has no internet access | Use the offline scene pack from this repository (already localized); do not use the un-collected development scenes |

**Conflicting drivers**: if a driver was already installed (for example via Ubuntu's "Software & Updates" → Additional Drivers), installing the `.run` driver from this document can leave two of them in place. Check:

```bash
ls /etc/vulkan/icd.d/ | grep nvidia
ls /usr/share/vulkan/icd.d/ | grep nvidia
dpkg -l | grep -i nvidia
```

If `nvidia_icd.json` shows up in both directories and `dpkg -l` lists system packages such as `nvidia-driver-xxx` or `libnvidia-*-xxx`, both the distro driver and the manual `.run` driver are installed. Remove the distro one and keep only the `.run` driver:

```bash
sudo apt purge '^nvidia-.*' '^libnvidia-.*' '^xserver-xorg-video-nvidia-.*' \
  '^linux-modules-nvidia-.*' '^linux-objects-nvidia-.*' '^linux-signatures-nvidia-.*' \
  '^screen-resolution-extra$'
sudo apt autoremove
sudo reboot
```

Run `nvidia-smi` again after rebooting to confirm the driver still works. This does not touch the `.run` driver, which is not managed by apt.

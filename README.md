<h1 align="center">Holydrone Cali</h1>

<hr>

<h3 align="center">A Camera–IMU Calibration Toolkit for UAVs</h3>

<p align="center">
  <img src="https://img.shields.io/badge/ROS-Noetic-22314E?style=for-the-badge&amp;logo=ros" alt="ROS Noetic">
  <img src="https://img.shields.io/badge/Ubuntu-20.04-E95420?style=for-the-badge&amp;logo=ubuntu" alt="Ubuntu 20.04">
  <img src="https://img.shields.io/badge/C%2B%2B-14-00599C?style=for-the-badge" alt="C++14">
  <img src="https://img.shields.io/badge/Python-3-3776AB?style=for-the-badge&amp;logo=python" alt="Python 3">
  <img src="https://img.shields.io/badge/Cameras-OV2710%20%7C%20IMX577-374151?style=for-the-badge" alt="OV2710 and IMX577">
</p>

<p align="center">
  <a href="#overview">Overview</a> ·
  <a href="#installation">Installation</a> ·
  <a href="#cameras">Cameras</a> ·
  <a href="#calibration">Calibration</a> ·
  <a href="#validation">Validation</a> ·
  <a href="#acknowledgments">Acknowledgments</a>
</p>

<p align="center">相机内参 · 相机–IMU 时空外参 · IMU 噪声 · ROS1 相机驱动</p>

<hr>

<a id="overview"></a>

## Overview · 项目概览

基于 ROS Noetic、Kalibr 和 allan_variance_ros，完成单目内参、相机–IMU 外参与 IMU 噪声标定。
新采集数据及结果保存到 `outputs/`；外部 bag 的 Kalibr 结果保存到 bag 同目录，该目录须可写。

| 功能 | 工具 / 入口 | 主要输出 |
| --- | --- | --- |
| 单目内参 | Kalibr · `_scripts/cali_intrinsics.sh` | 相机模型、焦距、主点、畸变、报告 |
| 相机–IMU 外参 | Kalibr · `_scripts/cali_extrinsics.sh` | 刚体变换、时间偏移、报告 |
| IMU 噪声 | allan_variance_ros · `_scripts/cali_imu.sh` | Allan 曲线、噪声密度、随机游走 |
| 在线图像采集 | OV2710 / IMX577 独立 ROS 包 | RGB8 图像、MJPEG、CameraInfo |

仓库保存代码、配置和标定板；原始录包、标定结果、日志与第三方构建产物均不纳入 Git。

<a id="installation"></a>

## Installation · 环境安装

Ubuntu 20.04 / ROS Noetic / Python 3。以下命令均在仓库根目录执行。

使用系统包安装：

```bash
sudo apt install libturbojpeg0-dev python3-catkin-tools python3-yaml v4l-utils \
  ros-noetic-camera-info-manager ros-noetic-image-transport \
  ros-noetic-mavros ros-noetic-rosbag ros-noetic-rqt-image-view \
  ros-noetic-topic-tools
```

### Kalibr

按 [Kalibr 上游说明](https://github.com/ethz-asl/kalibr) 安装依赖后编译：

```bash
# 创建 Kalibr 的 Catkin 工作空间源码目录
mkdir -p third_party/kalibr/src
# 下载 Kalibr 源码
git clone https://github.com/ethz-asl/kalibr.git third_party/kalibr/src/kalibr
# 切换到本项目验证过的固定版本
git -C third_party/kalibr/src/kalibr checkout 1f60227442d25e36365ef5f72cd80b9666d73467
# 应用本机 cv_bridge 导入顺序兼容补丁（首次安装执行一次）
git -C third_party/kalibr/src/kalibr apply "$PWD/patches/kalibr_cv_bridge_import.patch"
# 加载 ROS Noetic 环境
source /opt/ros/noetic/setup.bash
# 以 Release 模式编译 Kalibr 工作空间
catkin build --workspace "$PWD/third_party/kalibr" --cmake-args -DCMAKE_BUILD_TYPE=Release
```

补丁仅将 `cv2` 提前到 `cv_bridge` 之前导入，不改变 Kalibr 标定算法。
本机已经安装完成时，可直接使用已有工作空间，无需重复下载和应用补丁。

### Allan 方差

按 [allan_variance_ros 上游说明](https://github.com/ori-drs/allan_variance_ros) 安装依赖后编译：

```bash
mkdir -p third_party/allan/src
git clone https://github.com/ori-drs/allan_variance_ros.git third_party/allan/src/allan_variance_ros
git -C third_party/allan/src/allan_variance_ros checkout 1d54b602ee7f2ba0427865d63afe4945d913ed24
source /opt/ros/noetic/setup.bash
catkin build --workspace "$PWD/third_party/allan" allan_variance_ros
```

<a id="cameras"></a>

## Cameras · 相机驱动

当前默认使用 OV2710 的 UVC MJPEG 驱动；已有 bag 时可跳过此节。
OV2710 型号由用户提供，USB 标识本身不能确认传感器型号。

```bash
source /opt/ros/noetic/setup.bash
catkin build --workspace "$PWD/camera_ws" camera_driver_ov2710 camera_driver_imx577
```

两台相机使用独立 ROS 包及配置，均位于 `camera_ws/src/`：

| 相机包 | 配置文件（包内 `config/`） | 采集模式 |
| --- | --- | --- |
| `camera_driver_ov2710`（默认） | `camera_ov2710_1920x1080.yaml` | 1920×1080 · 30 fps · MJPEG |
| `camera_driver_imx577` | `camera_imx577_2592x1944.yaml` | 2592×1944 · 30 fps · MJPEG |

OV2710 包兼容 JPEG 结束标记后的填充字节，避免将可解码帧误判为坏帧。
处理流程见 [OV2710 包说明](camera_ws/src/camera_driver_ov2710/README.md)。

两份配置均沿用 180° 旋转和 `/camera/image_raw` 话题。
设备节点、分辨率和旋转应按实际相机设置；配置帧率不代表实测发布帧率。
单独启动当前 USB 相机：

```bash
source camera_ws/devel/setup.bash
roslaunch camera_driver_ov2710 parallel_mjpeg_camera.launch
```

切换到原 IMX577 配置：

```bash
roslaunch camera_driver_imx577 parallel_mjpeg_camera.launch
```

内参、外参脚本默认使用 OV2710 的 1920×1080 配置。
切换到 IMX577 时同时切换驱动包，默认加载它自己的配置：

```bash
CAMERA_PACKAGE=camera_driver_imx577 bash _scripts/cali_intrinsics.sh
```

同一驱动下也可通过 `CAMERA_PARAM_FILE` 指定配置文件绝对路径。
该变量非空时向 launch 传入 `camera_params_file`；更换为不支持此参数的驱动时留空。

更换相机时，编译对应驱动包，提供默认配置齐全的 launch 和 `sensor_msgs/Image` 话题。
在 `_scripts/common.sh` 修改以下变量，或运行时指定：

```bash
CAMERA_PACKAGE=my_camera_driver CAMERA_LAUNCH=camera.launch \
CAM_TOPIC=/my_camera/image_raw bash _scripts/cali_intrinsics.sh
```

内参、外参脚本共用这三个变量；驱动特有参数由对应 launch 管理。

### 确认相机输出

UVC 相机先检查设备及支持的格式、分辨率和帧率（按实际设备替换 `/dev/video0`）：

```bash
ls /dev/video*
v4l2-ctl -d /dev/video0 --list-formats-ext
v4l2-ctl -d /dev/video0 --get-fmt-video
```

启动驱动后，在另一终端加载 ROS 环境并检查图像话题；更换相机时替换为对应话题：

```bash
source /opt/ros/noetic/setup.bash
rostopic list
rostopic hz /camera/image_raw
rostopic echo -n 3 /camera/image_raw/header
rosrun rqt_image_view rqt_image_view /camera/image_raw
```

确认帧率稳定且接近配置值，时间戳非零并持续递增，图像清晰、曝光正常且旋转方向正确。
`rostopic hz` 持续运行，按 Ctrl-C 结束后执行下一条命令。

<a id="calibration"></a>

## Calibration · 标定流程

```mermaid
flowchart LR
    A[准备标定板] --> B[相机内参 + IMU Allan]
    B --> C[相机–IMU 外参]
    C --> D[结果验收]
```

### AprilGrid 标定板

配套文件位于 `targets/`：

- [打印 PDF](targets/aprilgrid_6x6_25mm_spacing03.pdf)
- [标定板配置](targets/aprilgrid_6x6_25mm_spacing03.yaml)（脚本默认使用）

参数为 6×6、Tag 边长 25 mm、间隔比例 0.3（间隙 7.5 mm）。可直接打印 PDF，
也可用 Kalibr 重新生成；命令末尾指定输出文件名前缀：

```bash
source third_party/kalibr/devel/setup.bash
mkdir -p outputs
rosrun kalibr kalibr_create_target_pdf \
  --type apriltag --nx 6 --ny 6 --tsize 0.025 --tspace 0.3 --tfam t36h11 \
  "$PWD/outputs/aprilgrid_6x6_25mm_spacing03"
```

生成工具依赖 PyX 和 TeX，缺失时按报错安装对应依赖，
或直接使用已有 PDF。打印选择 **100% 原始比例**，关闭“适应页面”；实测 Tag 黑色外边框边长应为25 mm，
相邻 Tag 间隙应为7.5 mm，打印后固定在平整硬板上。更换尺寸时同步修改目标 YAML。

### IMU 噪声标定

运行 `bash _scripts/cali_imu.sh` 采集，或在命令末尾添加已有静置 bag 路径。
设置脚本中的 `IMU_CALIBRATION_DURATION`（秒），并同步 `configs/imu_allan.yaml` 的时长与实际采样率。
正式标定至少静置采集 3 小时；输出 `outputs/imu/imu_mavros.yaml`。
当前 `configs/imu_allan.yaml` 的 `sequence_time: 200` 为短时流程测试配置；
正式采集前将其改为 `10800`，并确认脚本录制时长同为 `10800` 秒。

### 内参标定

运行 `bash _scripts/cali_intrinsics.sh` 采集，或在命令末尾添加已有图像 bag 路径。
默认模型 `pinhole-equi`；`CAMERA_MODEL` 可切换模型，`KALIBR_BAG_FREQ` 可设置分析抽样频率。

操作技巧：

- 固定相机，移动 AprilGrid，使角点覆盖画面中心、四边和四角。
- 改变距离和倾角，缓慢移动、换姿态后稍停，避免重复画面、模糊、反光和过曝。
- 标定板平整，实测尺寸与目标 YAML 一致；固定对焦，分辨率、ROI 和旋转与实际使用一致。

### 外参标定

编辑 `_scripts/cali_extrinsics.sh` 顶部的 `CAMCHAIN`、`IMU`，填写内参结果和实测 IMU 噪声参数。
`BAG` 留空录制，填写绝对路径则离线分析；其他参数也在顶部设置。

```bash
bash _scripts/cali_extrinsics.sh
```

固定标定板，移动刚性连接的相机与 IMU，充分激励三轴旋转和平移，保持图像清晰。
采集时按 Ctrl-C 结束录包并开始分析；优化时按 Ctrl-C 中止。
IMU 和内参脚本选择 `n` 录新包；内参脚本支持 `--help`，外参脚本直接使用顶部配置。

### 坐标系与结果使用

Kalibr 的 `T_cam_imu` 将 IMU 坐标转换到相机坐标：
`p_cam = T_cam_imu * p_imu`；时间偏移满足
`t_imu = t_cam + timeshift_cam_imu`。
相机光学坐标通常为右、下、前，IMU 坐标以消息实际定义为准。

内参结果只适用于标定时的镜头、分辨率、ROI、对焦和图像旋转设置。
外参质量结合重投影误差、残差和多次独立标定的重复性判断。
MJPEG 话题发布原始方向的 JPEG，`image_raw` 应用配置中的旋转；
标定时始终使用与部署一致的图像话题。

<a id="validation"></a>

## Validation · 测试验证

```bash
source /opt/ros/noetic/setup.bash
catkin run_tests --workspace "$PWD/camera_ws" camera_driver_ov2710
catkin_test_results camera_ws/build/camera_driver_ov2710/test_results
for script in _scripts/*.sh; do bash -n "$script" || break; done
shellcheck -S error _scripts/*.sh
```

2026-11-16 在 Jetson NX 上完成的验证：

| 检查 | 结果 |
| --- | --- |
| OV2710、IMX577 驱动编译 | 两包通过 |
| JPEG 边界及填充处理 | 4 项测试通过 |
| OV2710 实机采集 | 1920×1080 RGB8，约 29.8 fps |
| 12 秒采集检查 | 收到 330 帧，时间戳递增；坏帧、流重启均为 0 |
| 压缩图边界 | 330 / 330 帧通过 |
| Kalibr 固定版本 | 完整构建通过，内外参入口可加载，已有 bag 首帧可读 |

这些检查验证驱动和依赖能运行；完整内参、外参优化与三小时 IMU 噪声标定
仍需按实际硬件和采集数据执行。IMX577 本轮仅验证编译与配置加载。

<a id="acknowledgments"></a>

## Acknowledgments · 致谢与许可

- [Kalibr](https://github.com/ethz-asl/kalibr)：相机及视觉惯性标定。
- [allan_variance_ros](https://github.com/ori-drs/allan_variance_ros)：IMU Allan 方差分析。
- [ROS](https://www.ros.org/) 与 [libjpeg-turbo](https://github.com/libjpeg-turbo/libjpeg-turbo)：消息传输与 JPEG 解码。

第三方组件遵循各自许可证；Kalibr 补丁随附其固定版本的
[上游许可文本](patches/KALIBR-LICENSE)。相机包保留原有 BSD-3-Clause 声明；
仓库级自有代码许可证尚未单独指定。

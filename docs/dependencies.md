# 依赖与可复现环境

参考系统为 Ubuntu 20.04 + ROS Noetic。相机驱动要求 libturbojpeg、
camera_info_manager、image_transport、roscpp、sensor_msgs。
运行脚本需要 rosbag、MAVROS、rqt_image_view、topic_tools、Python 3 和 PyYAML。
使用系统包安装：

```bash
sudo apt install libturbojpeg0-dev python3-catkin-tools python3-yaml \
  ros-noetic-camera-info-manager ros-noetic-image-transport \
  ros-noetic-mavros ros-noetic-rosbag ros-noetic-rqt-image-view \
  ros-noetic-topic-tools
```

## Kalibr

上游：https://github.com/ethz-asl/kalibr
本机基线 commit：`1f60227442d25e36365ef5f72cd80b9666d73467`。
按上游文档安装系统依赖；此处仅给出源码定位和补丁步骤：

```bash
mkdir -p third_party/kalibr/src
git clone https://github.com/ethz-asl/kalibr.git third_party/kalibr/src/kalibr
git -C third_party/kalibr/src/kalibr checkout 1f60227442d25e36365ef5f72cd80b9666d73467
git -C third_party/kalibr/src/kalibr apply --check "$PWD/patches/kalibr-local.patch"
git -C third_party/kalibr/src/kalibr apply "$PWD/patches/kalibr-local.patch"
source /opt/ros/noetic/setup.bash
catkin build --workspace "$PWD/third_party/kalibr" --cmake-args -DCMAKE_BUILD_TYPE=Release
```

补丁从本机实际使用的 Kalibr 修改导出，覆盖 spline knot 配置、
时间偏移搜索、鲁棒损失、rolling shutter 行延迟、IMU bag 时间选项、
图像读取与报告兼容性等。原始补丁缺失问题由这份完整差异替代。
如出现编译依赖错误，先按固定版本的上游 README 补齐依赖。

`KALIBR_FOCAL_LENGTH_INIT` 可覆盖内参初始焦距（像素）。
需要时设定 `KALIBR_BAG_FREQ` 降低检测采样频率。
rolling shutter 行延迟必须对应发布图像的行方向和旋转设置，
未知时保持 0；这是实验选项，不能从相机型号直接猜测。

## Allan 方差

上游：https://github.com/ori-drs/allan_variance_ros
本机使用 commit：`1d54b602ee7f2ba0427865d63afe4945d913ed24`。

```bash
mkdir -p third_party/allan/src
git clone https://github.com/ori-drs/allan_variance_ros.git third_party/allan/src/allan_variance_ros
git -C third_party/allan/src/allan_variance_ros checkout 1d54b602ee7f2ba0427865d63afe4945d913ed24
source /opt/ros/noetic/setup.bash
catkin build --workspace "$PWD/third_party/allan" allan_variance_ros
```

按照上游说明安装分析脚本的 Python 依赖。运行脚本前需确认
`imu_rate`、`measure_rate`、`sequence_time` 与录制数据一致。

可以复用已构建的外部工作空间，例如：

```bash
KALIBR_WS=/absolute/path/kalibr \
bash scripts/cali_intrinsics.sh /absolute/path/camera.bag
```

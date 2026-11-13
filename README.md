# holydrone_cali

无人机单目相机内参、相机–IMU 外参和 IMU Allan 方差标定工具。
基于 ROS Noetic、Kalibr、allan_variance_ros，附带 UVC MJPEG 相机驱动。

仓库仅保存代码、配置模板、标定板配置和依赖补丁。
所有录包、参数结果、CSV、图像、报告及日志均写入 Git 忽略的 `outputs/`。
模板中的 IMU 噪声数值仅为示例，不能直接当作设备标定结果。

## 环境

参考环境：Ubuntu 20.04 / ROS Noetic / Python 3。依赖安装和
Kalibr 固定版本、修改说明见 [docs/dependencies.md](docs/dependencies.md)。
这不是一套新的标定算法；脚本负责采集、校验和调用上游工具。

## 相机驱动

在仓库根目录执行：

```bash
source /opt/ros/noetic/setup.bash
catkin build --workspace "$PWD/camera_ws" camera_driver
```

编辑 `configs/camera.yaml` 中的设备、分辨率、帧率和图像旋转。
默认是 IMX577 的 2592×1944 MJPEG 示例，其他 UVC 相机应按实际支持的模式修改。
相机发布话题默认是 `/camera/image_raw`。
相机驱动 launch 必须传入配置文件：

```bash
source camera_ws/devel/setup.bash
roslaunch camera_driver parallel_mjpeg_camera.launch camera_params_file:="$PWD/configs/camera.yaml"
```

如果已有图像/IMU rosbag，可以直接运行离线标定，无需编译相机驱动。

## 标定流程

1. IMU 静置采集并计算 Allan 方差：
   `bash scripts/cali_imu.sh`。默认记录 3 小时；
   或 `bash scripts/cali_imu.sh /absolute/path/stationary.bag`。
   输出噪声参数是 `outputs/imu/imu_mavros.yaml`。
   根据实际采样率修改 `configs/imu_allan.yaml`。
2. 相机内参：`bash scripts/cali_intrinsics.sh`；
   或 `bash scripts/cali_intrinsics.sh /absolute/path/camera.bag`。
   默认模型为 `pinhole-equi`，结果位于输入 bag 同目录。
3. 相机–IMU 外参，明确指定上一步的 camchain 和真实 IMU 噪声参数：

```bash
IMU_CONFIG="$PWD/outputs/imu/imu_mavros.yaml" \
bash scripts/cali_extrinsics.sh \
  --camchain "$PWD/outputs/intrinsics/<name>-camchain.yaml"
```

命令末尾添加已有外参 bag 路径即可离线重跑。
所有脚本均支持 `--help`。交互采集时选择 `n` 录制新包，
按 Ctrl-C 结束采集并开始分析；在优化阶段按 Ctrl-C 可中止优化。
IMU 静置不足配置时长会被数据校验拒绝。

Kalibr 对外部输入 bag 的结果通常写在 **bag 所在目录**，该目录必须可写。
如需所有输出集中存放，请先将待分析 bag 放在 `outputs/` 下。

## 可配置项

使用环境变量覆盖默认值；路径建议全部填写绝对路径。

| 变量 | 默认 |
| --- | --- |
| ROS_SETUP | /opt/ros/noetic/setup.bash |
| CAMERA_WS | 仓库下 camera_ws |
| KALIBR_WS | 仓库下 third_party/kalibr |
| ALLAN_WS | 仓库下 third_party/allan |
| OUTPUT_ROOT | 仓库下 outputs |
| CAMERA_PARAM_FILE | configs/camera.yaml |
| TARGET | targets/aprilgrid_6x6_25mm_spacing03.yaml |
| FCU_URL | /dev/ttyTHS0:2000000 |
| CAM_TOPIC | /camera/image_raw |
| IMU_TOPIC | /mavros/imu/data |
| IMU_CONFIG | configs/imu_mavros.example.yaml（外参时务必替换） |
| ALLAN_CONFIG | configs/imu_allan.yaml |
| CAMERA_MODEL | pinhole-equi |

修改话题后，还须同步修改 camera.yaml、IMU YAML 和 Allan YAML 对应字段；
CAM_TOPIC 只指定采集和分析话题，不会自动重命名驱动输出。
更换设备、分辨率、ROI、镜头焦距、对焦或图像旋转后需要重新标定。

## 采集与坐标系

AprilGrid 为 6×6，tag 边长 25 mm，间隔比例 0.3。
用 Kalibr 的 `kalibr_create_target_pdf --help` 查看打印工具选项；
打印后用尺子核对实际尺寸，并固定在平整硬板上。

内参采集覆盖整个画面，改变距离和倾斜角，避免过曝和运动模糊。
外参采集时固定标定板，移动刚性连接的相机和 IMU，
平滑激励三轴旋转与平移，起止各静止数秒。脚本提供角速度提示。
质量阈值只是经验参考，需要结合报告、残差和多次独立采集判断。

Kalibr 的 `T_cam_imu` 将 IMU 坐标转换到相机坐标：
`p_cam = T_cam_imu * p_imu`。
相机光学坐标通常为右、下、前；IMU 坐标以消息真实轴定义为准，
不要把飞控 FRD、ROS FLU 和相机光学坐标混用。
时间偏移满足 `t_imu = t_cam + timeshift_cam_imu`。

## 检查

```bash
for script in scripts/*.sh; do bash -n "$script" || break; done
shellcheck -S error scripts/*.sh
python3 scripts/monitor_imu_angular_speed.py --help
git status --ignored --short
```

上游版权与依赖见 [THIRD_PARTY.md](THIRD_PARTY.md)。
本项目自有代码的开源许可证尚待仓库所有者选择。

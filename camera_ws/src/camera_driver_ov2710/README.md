# camera_driver_ov2710

沿用 IMX577 驱动的 ROS1 / V4L2 mmap 采集、多线程 TurboJPEG 解码与发布结构，
用于当前 Sonix/Microdia USB 相机。OV2710 名称由用户提供，USB ID
`0c45:6368` 本身不能确认传感器型号。

输入为 UVC MJPEG。采集线程校验 JPEG 起止标记，去除 EOI 后的填充字节，
再发布压缩图并交给解码线程；缺失标记的帧仍会被拒绝。发布线程选择最新图像，
避免积压与乱序。配置默认 1920×1080、30 fps、180° 旋转；
实际帧率以统计日志和话题测量为准。

- `/camera/image_raw`：旋转后的 RGB8 图像，供 Kalibr 使用。
- `/camera/image_raw/mjpeg`：去除尾部填充的原始 JPEG，未旋转。
- `/camera/camera_info`：随解码图像发布；本包不计算内参，
  `camera_info_url` 未加载标定结果时使用未标定信息。

在仓库根目录执行：

```bash
source /opt/ros/noetic/setup.bash
catkin build --workspace "$PWD/camera_ws" camera_driver_ov2710
source camera_ws/devel/setup.bash
roslaunch camera_driver_ov2710 parallel_mjpeg_camera.launch
```

默认配置为 `config/camera_ov2710_1920x1080.yaml`。
两种相机包使用相同 ROS 节点名和设备，需分别启动。
标记处理测试：`catkin run_tests --workspace "$PWD/camera_ws" camera_driver_ov2710`。

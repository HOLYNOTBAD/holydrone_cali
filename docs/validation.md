# 本地验证

验证日期：2026-11-13。工作目录：/home/holy/holydrone_cali。

- 相机驱动在独立 catkin workspace 编译成功：2 packages succeeded，
  未链接原 bvr 工作空间。
- roslaunch --dump-params 成功解析独立 launch 和相机配置模板。
- 三个标定脚本 --help 和角速度监控 --help 均正常退出。
- bash -n 与 shellcheck -S error 检查通过。
- Kalibr 补丁在原本机修改源码上 git apply --reverse --check 通过，
  证明补丁与本机修改一致。
- 无效 bag 路径会提前拒绝；不会启动采集或优化。
- Git 忽略 outputs、相机驱动构建空间、第三方源码和 rosbag。

本轮未启动真实硬件采集、未执行三小时 IMU 录制，也未运行完整
Kalibr 优化。编译和静态检查不能替代实际硬件标定质量验证。

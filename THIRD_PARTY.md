# 第三方来源

- Kalibr：https://github.com/ethz-asl/kalibr 。
  此固定版本的许可证含广告署名条款，详见 patches/KALIBR-LICENSE；
  不应将其简化为 BSD-3-Clause。
  patches/kalibr-local.patch 基于 commit
  1f60227442d25e36365ef5f72cd80b9666d73467 的本地修改，
  包含上游代码上下文；保留其版权声明，见 patches/KALIBR-LICENSE。
  完整依赖树内各组件以各自许可证为准。
- allan_variance_ros：https://github.com/ori-drs/allan_variance_ros 。
  本仓库调用外部安装，不分发其源码；许可证以固定版本上游为准。
- camera_driver 和采集脚本来自 HOLYNOTBAD 的 bvr 工作区。
  camera_driver/package.xml 保留原有 BSD-3-Clause 声明；
  其他项目自有代码的仓库级许可证待所有者选择。
  ROS 与 libturbojpeg 为外部依赖。

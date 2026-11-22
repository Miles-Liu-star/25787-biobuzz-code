# Pinpoint XY 读取程序

本文件夹整理了 Pinpoint XY 读取器及其项目内依赖，便于集中查看和复用。

## 文件

- `PinpointXYReader.java`：读取器 API，提供 `getXmm()`、`getYmm()`、`getHeading()` 和 `getPositionMm()`；航向单位为弧度。
- `PinpointXYTeleOp.java`：在 Driver Station 上实时显示 XY 的调试 OpMode。
- `PinpointPose.java`：Pinpoint 初始化、采样、坐标锚定和读数恢复。
- `RobotConfig.java`：Pinpoint 硬件名、里程轮方向、偏移和机器人默认起始尺寸配置。
- `Maths.java`：定位算法使用的数学及时间工具。

坐标约定：X 向前、Y 向左，单位为毫米。程序还依赖 FTC SDK 中的硬件和 OpMode API。

这里的 Java 文件是集中整理的副本；Android Studio 实际编译使用的源文件仍位于
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/`。修改定位代码后请同步更新副本。

# FRC26

FRC 2026 赛季，10016 队机器人代码。使用 Java 17、WPILib / GradleRIO，包含 Swerve 底盘、PathPlanner 自动路径与三路 Limelight AprilTag 视觉融合定位。

## 开发环境

- Java 17
- WPILib 2026（本项目使用 GradleRIO 2026.2.1）
- 项目 vendordeps 中声明的厂商依赖

Windows 下使用项目自带的 Gradle Wrapper：

```powershell
.\gradlew.bat build
.\gradlew.bat test
.\gradlew.bat deploy
```

部署队号由 `.wpilib/wpilib_preferences.json` 配置。部署前请核对硬件接线、CAN ID、绝对编码器偏移与 `Constants.java` 中的速度设置。当前未默认启用桌面仿真 JNI 支持。

## 代码结构

- `src/main/java/frc/robot/Robot.java`：机器人生命周期与硬件、自动程序接线。
- `src/main/java/frc/robot/subsystems/chassis/`：底盘驱动、里程计与视觉融合。
- `src/main/java/frc/robot/command/Auto.java`：PathPlanner 自动程序配置。
- `src/main/deploy/pathplanner/`：PathPlanner 配置、路径与自动程序。
- `AGENTS.md`：架构、开发约定与注意事项。

Swerve 模块数组顺序为 `{lf, rf, lb, rb}`。里程计在独立线程运行，访问位姿估计器时需要遵循现有同步约定。

## 许可证

本项目原创代码采用 [MIT License](LICENSE)。WPILib 相关授权见 [WPILib-License.md](WPILib-License.md)，第三方代码及依赖遵循各自许可证。

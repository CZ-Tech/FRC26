// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.command.Auto;
import frc.robot.lib.LimelightHelpers;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.MagicIntaker;
import frc.robot.subsystems.chassis.ChassisSwerveOdo;
import frc.robot.subsystems.chassis.OdoSwerveDrivetrain;
import frc.robot.subsystems.chassis.SwerveChassis;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  public static volatile Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue); 

  private final PS5Controller gamepad0; 
  private final PS5Controller gamepad1; 
  private final Field2d fieldpos = new Field2d(); 

  private final OdoSwerveDrivetrain driveTrain;
  private final ChassisSwerveOdo odo;
  private final Shooter shooter = new Shooter();
  private final Transfer transfer = new Transfer();
  private final MagicIntaker intaker = new MagicIntaker();
  private Auto auto; 

  private Command autoCommand; 

  /**
   * 如果没有任何调用父类需求，那么初始化代码应放这里
   */
  public Robot() { 
    System.out.println("RobotIniting1"); 
    AHRS gyro = new AHRS(NavXComType.kMXP_SPI); 
    gyro.reset(); 
    gamepad0 = new PS5Controller(0); 
    gamepad1 = new PS5Controller(1); 

    SwerveChassis modules = SwerveChassis.getInstance(); // 初始化四个矢量轮模块 
    odo = new ChassisSwerveOdo(modules, gyro); 
    driveTrain = new OdoSwerveDrivetrain(modules, odo); 
    SmartDashboard.putBoolean("auto", false); 

    intaker.resetEncoderToUp();
  }

  @Override
  public void robotInit() {
    System.out.println("RobotIniting2");
    // (robotIP):5801 will now point to a Limelight3A's (id 0) web interface stream:
    // (robotIP):5800 will now point to a Limelight3A's (id 0) video stream:
    LimelightHelpers.setupPortForwardingUSB(0); // First camera
    LimelightHelpers.setupPortForwardingUSB(1); // Second camera

    CameraServer.startAutomaticCapture();

    // // Set a custom crop window for improved performance 设置自定义裁剪窗口以提高性能 (-1 to 1
    // for each value)
    // LimelightHelpers.setCropWindow("", -1, 1, -1, 1);
    // 设置Limelight到机器人中心的位置以便AprilTag跟踪计算中心坐标 (meters)
    // LimelightHelpers.setFiducial3DOffset("",
    // 0.0, // Forward offset
    // 0.0, // Side offset
    // 0.0 // Height offset
    // );
    // // Adjust keystone crop window 调整梯形裁切窗口 (-0.95 to 0.95 for both horizontal
    // and vertical)
    // LimelightHelpers.setKeystone("", 0.1, -0.05);

    // // 设置AprilTag定位参数 TODO 需要在网页面板设置！！！
    // LimelightHelpers.SetFiducialIDFiltersOverride("", new int[]{1, 2, 3, 4}); //
    // 只跟踪这些AprilTag
    // LimelightHelpers.SetFiducialDownscalingOverride("", 2.0f); // 半分辨率处理

    // Change the camera pose relative to robot center (x forward, y left, z up,
    // degrees)
    // LimelightHelpers.setCameraPose_RobotSpace("",
    // Constants.LimelightXOffset, // Forward offset (meters)
    // Constants.LimelightYOffset, // Side offset (meters)
    // Constants.LimelightZOffset, // Height offset (meters)
    // Constants.LimelightRollDeg, // Roll (degrees)（方向盘那么转）
    // Constants.LimelightPitchDeg, // Pitch (degrees)（俯仰角）
    // Constants.LimelightYawDeg // Yaw (degrees)（在水平面旋转）
    // );

    alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    // 初始化检查单
    // FIXME 如果面板上Prepared灯不绿，绝对绝对不能开始比赛！！！
    Checklist.allianceGot.set(() -> DriverStation.getAlliance().isPresent());
    Checklist.autoChosen.set(() -> autoCommand != null);
    Checklist.MotorsOK.set(() -> driveTrain.isMotorConnected());
    Checklist.shooterMotorsOK.set(() -> shooter.areMotorsConnected());
    Checklist.odoAccurate.set(() -> odo.millisFromLastTag() <= Constants.WarnOdoIncorrectMillis);
    Checklist.odoReady.set(() -> odo.isReady());
    Checklist.limelightFrontConnected.set(() -> odo.isLimelightFrontConnected());
    Checklist.limelightLeftConnected.set(() -> odo.isLimelightLeftConnected());
    Checklist.limelightRightConnected.set(() -> odo.isLimelightRightConnected());

    // 绑定自动事件并初始化
    auto = Auto.bindOperations()
        .add("onStart", () -> {
          System.out.println("AutoStarted");
          SmartDashboard.putBoolean("auto", true);
        })
        .add("onStop", () -> {
          System.out.println("AutoStoped");
          SmartDashboard.putBoolean("auto", false);
        })
        .add("base", () -> {
          System.out.println("approaching base");
        })
        .add("public_in", () -> {
          System.out.println("approaching public");
        })
        .add("public_out", () -> {
          System.out.println("leaving public");
        })
        .prepareAuto(odo, driveTrain);
    auto.prepareAutoChooser();
    System.out.println("RobotInit Init Finished! -------------------------");
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    CommandScheduler.getInstance().run(); // 执行全局调度器任务

    if (isDisabled())
      autoCommand = auto.getAuto(); // 赛前反复轮询自动路径

    Checklist.putAll(); // 执行检查单

    driveTrain.putMotorStatus(); // 发布单个电机状态到 SmartDashboard

    if (gamepad0.getOptionsButtonPressed() && gamepad0.getCreateButton()) {
      driveTrain.setNowForward();
    }
    SmartDashboard.putNumber("LimelightRotation", LimelightHelpers.getBotPose2d(Constants.LLNameFront).getRotation().getDegrees());
    SmartDashboard.putNumber("LimelightPitching", LimelightHelpers.getBotPose3d(Constants.LLNameFront).getRotation().getY());

    fieldpos.setRobotPose(odo.getBotPoseEstimate_wpiBlue());
    SmartDashboard.putData("FieldPos", fieldpos);

    driveTrain.putWheelHeadingDeg();
    // SmartDashboard.putNumber("rLF", (lf.getTurningEncoderRadians()));
    // SmartDashboard.putNumber("rRF", (rf.getTurningEncoderRadians()));
    // SmartDashboard.putNumber("rLB", (lb.getTurningEncoderRadians()));
    // SmartDashboard.putNumber("rRB", (rb.getTurningEncoderRadians()));

    // lift.putMotorPosition();
    intaker.putLog();
    // 自瞄：根据场地坐标始终跟踪目标角度
    // if (odo.isReady()) {
    //   shooter.autoAim(odo.getBotPoseEstimate_wpiBlue(), alliance);
    // }
    shooter.putData();
    transfer.putLog();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    autoCommand = auto.getAuto(); // 开赛前再同步一次选项
    if (autoCommand != null)
      CommandScheduler.getInstance().schedule(autoCommand);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // if (!autoCommand.isFinished()) {
    // autoCommand.execute();
    // } 在robotPeriodic里写了run了
  }

  @Override
  public void autonomousExit() {
    if (autoCommand != null)
      CommandScheduler.getInstance().cancel(autoCommand);
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    if (autoCommand != null)
      CommandScheduler.getInstance().cancel(autoCommand);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double[] gamepadL = new double[] {
        -Constants.zeroIfZero(gamepad0.getLeftY(), Constants.joystickDummyIgnore),
        -Constants.zeroIfZero(gamepad0.getLeftX(), Constants.joystickDummyIgnore) };
    double[] gamepadR = new double[] {
        -Constants.zeroIfZero(gamepad0.getRightY(), Constants.joystickDummyIgnore),
        -Constants.zeroIfZero(gamepad0.getRightX(), Constants.joystickDummyIgnore) };

    driveTrain.driveRobotFieldCentricLocalAlliance(
    // driveTrain.driveRobot(
        Constants.toSpeed(gamepadL[0]),
        Constants.toSpeed(gamepadL[1]),
        Constants.toSpeed(gamepadR[1]));

    if ((gamepad0.getL2Axis() + 1) / 2 > 0.5) {
      shooter.setFlywheelPower(0.6);
      transfer.turn();
      transfer.up();
    }

    if (gamepad0.getL1Button() && gamepad0.getSquareButton()) {
      shooter.setFlywheelPower(0.2);
    } else if (gamepad0.getSquareButton()) {
      shooter.setFlywheelPower(0.6);
    }

    if (gamepad0.getL1Button() && gamepad0.getCircleButton()) {
      transfer.turnReverse();
    } else if (gamepad0.getCircleButton()) {
      transfer.turn();
    }

    if (gamepad0.getL1Button() && (gamepad0.getR2Axis() + 1) / 2 > 0.5) {
      intaker.intake(true);
    } else if ((gamepad0.getR2Axis() + 1) / 2 > 0.5) {
      intaker.intake();
    }

    if (gamepad0.getR1Button()) {
      intaker.toggle();
    }
    
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // double[] gamepadL = new double[] {
    // -Constants.zeroIfZero(gamepad0.getLeftY(), Constants.joystickDummyIgnore),
    // -Constants.zeroIfZero(gamepad0.getLeftX(), Constants.joystickDummyIgnore) };
    // double[] gamepadR = new double[] {
    // -Constants.zeroIfZero(gamepad0.getRightY(), Constants.joystickDummyIgnore),
    // -Constants.zeroIfZero(gamepad0.getRightX(), Constants.joystickDummyIgnore) };
    // driveTrain.driveRobot(
    // gamepadL[0],
    // gamepadL[1],
    // gamepadR[1]);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    shooterTest();
    // swerveTest();
    // transferTest();
    // intakerTest();
  }

  boolean isAutoMoving = false;
  public void intakerTest() {
    double power1 = Constants.zeroIfZero((gamepad0.getL2Axis() + 1) / 2 * 0.6, 0.05);
    double power2 = Constants.zeroIfZero((gamepad0.getR2Axis() + 1) / 2 * -0.6, 0.05);
    if (power2 != 0.0 || !isAutoMoving) {
      intaker.manualUpDown(power1);
      isAutoMoving = false;
    }
    if (power2 != 0.0) {
      intaker.manualUpDown(power2);
      isAutoMoving = false;
    }

    if (gamepad0.getTriangleButtonPressed()) {
      intaker.intake();
    }
    if (gamepad0.getTriangleButtonReleased()) {
      intaker.stopIntake();
    }
    if (gamepad0.getPOV() == 0) {
      isAutoMoving = true;
      intaker.up();
    } else if (gamepad0.getPOV() == 180) {
      isAutoMoving = true;
      intaker.down();
    }
  }

  public void swerveTest() {
    // 底盘驱动测试
    double[] gamepadL = new double[] {
        -Constants.zeroIfZero(gamepad0.getLeftY(), Constants.joystickDummyIgnore),
        -Constants.zeroIfZero(gamepad0.getLeftX(), Constants.joystickDummyIgnore) };
    double[] gamepadR = new double[] {
        -Constants.zeroIfZero(gamepad0.getRightY(), Constants.joystickDummyIgnore),
        -Constants.zeroIfZero(gamepad0.getRightX(), Constants.joystickDummyIgnore) };
    driveTrain.driveRobot(
        Constants.toSpeed(gamepadL[0]),
        Constants.toSpeed(gamepadL[1]),
        Constants.turnToSpeed(gamepadR[1]));
  }


  public void shooterTest() {
    // =============================
    // Shooter 手动测试（进入 Test 模式）
    // 在 SmartDashboard 中观察各项读数，用 gamepad0 逐项触发
    // =============================

    // 电机连接状态
    SmartDashboard.putBoolean("Shooter FlywheelAtTarget", shooter.isFlywheelAtTarget());
    SmartDashboard.putBoolean("Shooter VerticalAtTarget", shooter.isVerticalAtTarget());
    SmartDashboard.putBoolean("Shooter HorizontalAtTarget", shooter.isHorizontalAtTarget());

    // --- 飞轮测试 ---
    // Cross: 低速启动飞轮（20 RPS）
    if (gamepad0.getCrossButton()) {
      shooter.runFlywheels(10);
    }
    // Circle: 停止飞轮
    if (gamepad0.getCircleButton()) {
      shooter.stopFlywheels();
    }

    shooter.setFlywheelPower(gamepad0.getRightY());

    // --- 角度测试（DPAD 步进 ±2°）---
    // DPAD 上/下: 调整俯仰角
    if (gamepad0.getPOV() == 0) {
      shooter.setVerticalAngle(shooter.getVerticalAngleDeg() + 2.0);
    } else if (gamepad0.getPOV() == 180) {
      shooter.setVerticalAngle(shooter.getVerticalAngleDeg() - 2.0);
    }
    // DPAD 右/左: 调整水平角
    if (gamepad0.getPOV() == 90) {
      shooter.setHorizontalAngle(shooter.getHorizontalAngleDeg() + 2.0);
    } else if (gamepad0.getPOV() == 270) {
      shooter.setHorizontalAngle(shooter.getHorizontalAngleDeg() - 2.0);
    }

    if (gamepad0.getL1ButtonPressed()) {
      shooter.setVerticalAngle(Constants.ShooterVerticalMinDeg);
    }

    // Square: 编码器归零
    if (gamepad0.getSquareButtonPressed()) {
      shooter.zeroEncoders();
    }

    // Triangle: 急停（四个电机全部停止）
    if (gamepad0.getTriangleButtonPressed()) {
      shooter.stopAll();
    }

    // // --- 手动开环测试（微功率，安全）---
    // // L2 / R2: 俯仰电机手动正反转（功率 0.15）
    // double verticalPower = 0.15 * (0.5 * (gamepad0.getL2Axis() + 1.0))
    //     - 0.15 * (0.5 * (gamepad0.getR2Axis() + 1.0));
    // shooter.setVerticalAnglePower(verticalPower);

    // // 左摇杆 X: 水平电机手动正反转（功率 0.15）
    // double horizontalPower = 0.5 * Constants.zeroIfZero(gamepad0.getLeftX(), 0.1);
    // shooter.setHorizontalAnglePower(horizontalPower);
  }


  boolean transferUping = false;

  public void transferTest() {
    // =============================
    // Transfer 手动测试（进入 Test 模式）
    // 在 SmartDashboard 中观察各项读数，用 gamepad0 逐项触发
    // =============================

    transfer.putLog();

    // --- 拨球滚轮测试 ---
    // Cross: 开始拨球
    if (gamepad0.getCrossButton()) {
      transfer.turn();
    }
    // Circle: 停止拨球
    if (gamepad0.getCircleButton()) {
      transfer.stopTurn();
    }
    // Square: 反向拨球（排障）
    if (gamepad0.getSquareButton()) {
      transfer.turnReverse();
    }

    // --- 上传电机测试 ---
    // Triangle: 开始上传
    if (gamepad0.getTriangleButtonPressed()) {
      if (!transferUping) {
        transferUping = true;
        transfer.up();
      } else {
        transferUping = false;
        transfer.stopUp();
       }
    } 


    if (!transferUping) {
      // --- 手动开环测试 ---
      // L2: 滚轮功率（0 到 1）
      double turnPower = (gamepad0.getL2Axis() + 1.0) / 2.0;
      transfer.setTurnPower(turnPower);

      // R2: 上传功率（0 到 1）
      double upPower = (gamepad0.getR2Axis() + 1.0) / 2.0;
      transfer.setUpPower(upPower);
    }

    // Options: 全部急停
    if (gamepad0.getOptionsButtonPressed()) {
      transfer.stopAll();
    }
  }
  

  @Override
  public void testExit() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

}

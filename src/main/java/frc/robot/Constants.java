package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.SwerveModule.MotorDirection;

public class Constants {
    public static final double kPModuleTurningController = 0.2;
    public static final double kPModuleDriveController = 0.001;
    public static final double SpeedFactor = 0.2;  // FIXME (调试限速用)速度输出将乘这个数字，赛场上应设为1
    
    public static final double kMaxSpeedMetersPerSecond = 13;  // 车辆实际最大速度，输出功率为目标速度与最大速度之比，即越大越慢
    public static final double MaxTurnSpeedRadPerSecond = 4 * Math.PI;
    // TODO: 'EncoderDistancePerPulse' should be calculated based on the gearing and wheel diameter
    public static final double kDriveEncoderDistancePerPulse = 0.0700D; //0.947
    public static final double DriveEncoderRotationePerMetre = 22D;  // 驱动电机(注意是电机！！不是轮！)每行驶1m的转数

    public static final double joystickDummyIgnore = 0.02;  // 当手柄移动小于这个值时视为0
    public static long WarnOdoIncorrectMillis = 30_000L;  // 30s

    // 混合里程计参数
    public static int    leastTagStart = 2;  // 至少看到多少个apriltag才初始化里程计
    public static double VisionPosStandardDeviationFactor = 0.4;  // 视觉标准差（越小越可信）
    public static double SwervePosStandardDeviationFactor = 0.2;  // 矢量轮编码器里程计标准差（越小越可信）

    // ---------------------------  发射  ---------------------------
    // CAN IDs（TODO/FIXME：按实际接线修改）
    public static int ShooterTopFlywheelCANID    = 50;
    public static int ShooterBottomFlywheelCANID = 51;
    public static int ShooterVerticalAngleCANID  = 52;
    public static int ShooterHorizontalAngleCANID= 53;

    // 输出方向（true=反转；TODO/FIXME：实测后确认）
    public static boolean ShooterTopFlywheelInverted      = false;
    public static boolean ShooterBottomFlywheelInverted   = true; // 与上飞轮反向
    public static boolean ShooterVerticalAngleInverted    = true;
    public static boolean ShooterHorizontalAngleInverted  = true;

    // 电流限制（TODO/FIXME：按电机和减速比实测）
    public static double ShooterFlywheelStatorCurrentLimit = 20.0;
    public static double ShooterFlywheelSupplyCurrentLimit = 20.0;
    public static double ShooterAngleStatorCurrentLimit    = 20.0;
    public static double ShooterAngleSupplyCurrentLimit    = 20.0;

    // 飞轮速度闭环 PID/FF（Slot0，VelocityVoltage，TODO/FIXME：实车整定）
    public static double ShooterFlywheelKP = 0.0;
    public static double ShooterFlywheelKI = 0.0;
    public static double ShooterFlywheelKD = 0.0;
    public static double ShooterFlywheelKS = 0.0;
    public static double ShooterFlywheelKV = 0.12; // RPS 时典型起始值

    // 俯仰角度位置闭环 PID/FF + 重力补偿（TalonFX Slot0，PositionVoltage，TODO/FIXME：实车整定）
    // 说明：使用 PositionVoltage 而不是 MotionMagic；负载不大，靠 kP/kD + kG 抵消重力即可。
    public static double ShooterVerticalAngleKP = 5.0;
    public static double ShooterVerticalAngleKI = 0.0;
    public static double ShooterVerticalAngleKD = 0.01;
    public static double ShooterVerticalAngleKS = 0.0;
    public static double ShooterVerticalAngleKV = 0.0; // 位置环通常不需要 kV
    public static double ShooterVerticalAngleKG = 0.0; // 持续向上的重力补偿电压（TODO/FIXME：实测）

    // 编码器→角度系数（TODO/FIXME：按减速比和机械零点标定）
    // 含义：电机转数 × 该系数 = 机构角度（度）
    public static double ShooterVerticalEncoderToDeg   = 15.5585;
    public static double ShooterHorizontalEncoderToDeg = 33.6;

    // 角度限位（TODO/FIXME：按机械结构实测）
    public static double ShooterVerticalMinDeg   = 16.620761;
    public static double ShooterVerticalMaxDeg   =  75.0 - 2.0;
    public static double ShooterVerticalInitDeg = ShooterVerticalMinDeg - 2.0;  // 竖直方向调节电机初始化时应处在的角度
    public static double ShooterHorizontalMinDeg = -170.0; // 水平旋转硬限位(从零点向左)
    public static double ShooterHorizontalMaxDeg =  170.0; // 水平旋转硬限位从(零点向右)

    /** 水平旋转电机最大输出功率（0.0 ~ 1.0），限制闭环 PID 输出和开环控制 */
    public static double ShooterHorizontalMaxPower = 0.3;

    // 公差（TODO/FIXME：实车整定）
    public static double ShooterFlywheelVelocityToleranceRPS = 5.0;
    public static double ShooterAngleToleranceDeg            = 1.0;

    // 水平角度电机 PID（SparkMax REVLib 位置闭环，TODO/FIXME：实车整定）
    public static double ShooterHorizontalAngleKP = 7;
    public static double ShooterHorizontalAngleKI = 0.0;
    public static double ShooterHorizontalAngleKD = 0.1;
    public static double ShooterHorizontalAngleKIZone = 0.0;
    public static double ShooterHorizontalAngleKFF = 0.0;

    public static double goalFieldPosX_wpiBlue = 4.0296;  // meter
    public static double goalFieldPosY_wpiBlue = 4.0346;  // meter

    public static double FieldWidth = 8.069;    // meter
    public static double FieldLength = 16.541;  // meter

    // --------------------------- Shooter Limelight 环几何参数 -------------------------
    // 环中心位于机器人地面投影中心上方 523mm，中线上距中心靠后 123.868mm
    // 环半径 150mm，环平面为水平面
    // TODO/FIXME: 用 CAD 数据验证这些参数

    /** 环中心 forward 偏移 (m)，负值 = 机器人中心后方 */
    public static double ShooterLimelightRingCenterX = -0.123868;
    /** 环中心 side 偏移 (m)，0 = 中线上 */
    public static double ShooterLimelightRingCenterY = 0.0;
    /** 环中心高度 (m)，相对地面投影中心 */
    public static double ShooterLimelightRingCenterZ = 0.523;
    /** 环半径 (m) */
    public static double ShooterLimelightRingRadius = 0.150;
    /** 角度零点偏移 (deg)，用于校正编码器零位与机械零位的偏差 */
    public static double ShooterLimelightAngleOffsetDeg = 0.0;

    // --- Shooter Limelight 固定安装姿态（不随环旋转）---
    /** Limelight 安装姿态 Roll (deg)，绕 forward 轴旋转 */
    public static double ShooterLimelightFixedRollDeg = 0.0;
    /** Limelight 安装姿态 Pitch (deg)，上仰为正 */
    public static double ShooterLimelightFixedPitchDeg = 25.0;
    /** Limelight 安装姿态 Yaw (deg)，左偏为正 */
    public static double ShooterLimelightFixedYawDeg = 0.0;

    // --- Shooter 自瞄参数 ---
    /** 球门开口中心高度 (m)，相对地面（TODO/FIXME: 根据实际场地测量） */
    public static double ShooterAutoAimGoalHeightMeters = 0.0;
    /** 发射机构 pivot 高度 (m)，与 Limelight 等高（TODO/FIXME: 根据 CAD 确认） */
    public static double ShooterAutoAimShooterHeightMeters = 0.523;

    // --------------------------- intaker -------------------------
    public static int IntakerUpdownCANID = 40;
    public static int IntakerUpdown2CANID = 41;
    public static int IntakerTurningCANID = 42;
    public static int IntakerTurning2CANID = 43;

    // 输出方向（true=反转）
    public static boolean IntakerUpdownInverted  = false;
    public static boolean IntakerUpdown2Inverted = true;
    public static boolean IntakerTurningInverted  = false;
    public static boolean IntakerTurning2Inverted = true;

    public static double IntakerDownPos = -16;
    public static double IntakerUpPos = 0;
    public static double IntakerRunPower = -0.5;

    // --------------------------- transfer -------------------------
    public static int TransferTurnMotor1CANID = 30;
    public static int TransferTurnMotor2CANID = 31;
    public static int TransferUpMotorCANID   = 32;

    public static double TransferTurnPower = 0.2;
    public static double TransferUpPower   = 0.7;

    // --------------------------- 驱动轮参数 -------------------------------------------
    // 左前
    public static int            DriveChannelLF                =  10;
    public static int            TurnChannelLF                 =  14;
    public static int            EncoderPortLF                 =  0; 
    public static double         TurningEncoderOffsetRadiansLF =  2.565634; // -4.02;
    public static double         DriveGainLF                   =  1;
    public static MotorDirection TurnPowerSignLF               =  MotorDirection.REVERSE;
    public static MotorDirection TurnEncoderSignLF             =  MotorDirection.REVERSE;
    public static Translation2d m_lf = new Translation2d(0.55 / 2, 0.55 / 2); // 相对于底盘中心位置
                                                                              // 正的x表示靠前，正的y表示偏左。

    // 左后
    public static int            DriveChannelLB                =  11;
    public static int            TurnChannelLB                 =  15;
    public static int            EncoderPortLB                 =  2; 
    public static double         TurningEncoderOffsetRadiansLB =  -2.251475;  
    public static double         DriveGainLB                   =  1;
    public static MotorDirection TurnPowerSignLB               =  MotorDirection.FORWARD;
    public static MotorDirection TurnEncoderSignLB             =  MotorDirection.REVERSE;
    public static Translation2d m_lb = new Translation2d(-0.55 / 2, 0.55 / 2);

    // 右前
    public static int            DriveChannelRF                =  12;
    public static int            TurnChannelRF                 =  16;
    public static int            EncoderPortRF                 =  1; 
    public static double         TurningEncoderOffsetRadiansRF =  -0.029147; // -5.222;  // 2.128
    public static double         DriveGainRF                   =  1;
    public static MotorDirection TurnPowerSignRF               =  MotorDirection.FORWARD;
    public static MotorDirection TurnEncoderSignRF             =  MotorDirection.REVERSE;
    public static Translation2d m_rf = new Translation2d(0.55 / 2, -0.55 / 2);

    // 右后
    public static int            DriveChannelRB                =  13;
    public static int            TurnChannelRB                 =  17;
    public static int            EncoderPortRB                 =  3; 
    public static double         TurningEncoderOffsetRadiansRB =  2.286;// -4.881;
    public static double         DriveGainRB                   =  -1;
    public static MotorDirection TurnPowerSignRB               =  MotorDirection.FORWARD;
    public static MotorDirection TurnEncoderSignRB             =  MotorDirection.REVERSE;
    public static Translation2d m_rb = new Translation2d(-0.55 / 2, -0.55 / 2);
    // ----------------------------------------------------------------------------------
    
    // TODO 需要在网页面板设置 limelight参数(x forward, y left, z up(degrees))
    // public static double LimelightXOffset  = 0.5;  // Forward offset (meters)
    // public static double LimelightYOffset  = 0.0;  // Side offset (meters)
    // public static double LimelightZOffset  = 0.5;  // Height offset (meters)
    // public static double LimelightRollDeg  = 0.0;  // Roll (degrees)（像方向盘那么转）
    // public static double LimelightPitchDeg = 30.0; // Pitch (degrees)（俯仰角）
    // public static double LimelightYawDeg   = -90;  // Yaw (degrees)（在水平面旋转，左加右减）

    public static String LLNameFront = "limelight";
    public static String LLNameLeft  = "limelight-left";
    public static String LLNameRight = "limelight-right";



    public static double toPower(double speed) {
        return speed / kMaxSpeedMetersPerSecond;
    }

    public static double toSpeed(double power) {
        return power * kMaxSpeedMetersPerSecond * SpeedFactor;
    }

    public static double turnToSpeed(double turnPower) {
        return turnPower * MaxTurnSpeedRadPerSecond * SpeedFactor;
    }
    
    public static double zeroIfZero(double original, double tolorance) {
        if (Math.abs(original) <= tolorance) {
            return 0.0D;
        } else {
            return original;
        }
    }
    
}

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
    public static boolean ShooterVerticalAngleInverted    = false;
    public static boolean ShooterHorizontalAngleInverted  = false;

    // 电流限制（TODO/FIXME：按电机和减速比实测）
    public static double ShooterFlywheelStatorCurrentLimit = 60.0;
    public static double ShooterFlywheelSupplyCurrentLimit = 60.0;
    public static double ShooterAngleStatorCurrentLimit    = 40.0;
    public static double ShooterAngleSupplyCurrentLimit    = 40.0;

    // 飞轮速度闭环 PID/FF（Slot0，VelocityVoltage，TODO/FIXME：实车整定）
    public static double ShooterFlywheelKP = 0.0;
    public static double ShooterFlywheelKI = 0.0;
    public static double ShooterFlywheelKD = 0.0;
    public static double ShooterFlywheelKS = 0.0;
    public static double ShooterFlywheelKV = 0.12; // RPS 时典型起始值

    // 角度位置闭环 PID/FF + 重力补偿（Slot0，PositionVoltage，TODO/FIXME：实车整定）
    // 说明：使用 PositionVoltage 而不是 MotionMagic；负载不大，靠 kP/kD + kG 抵消重力即可。
    public static double ShooterAngleKP = 10.0;
    public static double ShooterAngleKI = 0.0;
    public static double ShooterAngleKD = 0.5;
    public static double ShooterAngleKS = 0.1;
    public static double ShooterAngleKV = 0.0; // 位置环通常不需要 kV
    public static double ShooterAngleKG = 0.5; // 持续向上的重力补偿电压（TODO/FIXME：实测）

    // 编码器→角度系数（TODO/FIXME：按减速比和机械零点标定）
    // 含义：电机转数 × 该系数 = 机构角度（度）
    public static double ShooterVerticalEncoderToDeg   = 1.0;
    public static double ShooterHorizontalEncoderToDeg = 1.0;

    // 角度限位（TODO/FIXME：按机械结构实测）
    public static double ShooterVerticalMinDeg   = -30.0;
    public static double ShooterVerticalMaxDeg   =  60.0;
    public static double ShooterHorizontalMinDeg = -270.0; // 水平旋转硬限位：从零点向左最多 270°
    public static double ShooterHorizontalMaxDeg =  270.0; // 水平旋转硬限位：从零点向右最多 270°

    // 公差（TODO/FIXME：实车整定）
    public static double ShooterFlywheelVelocityToleranceRPS = 5.0;
    public static double ShooterAngleToleranceDeg            = 1.0;

    public static double goalFieldPosX_wpiBlue = 4.0296;  // meter
    public static double goalFieldPosY_wpiBlue = 4.0346;  // meter

    public static double FieldWidth = 8.069;    // meter
    public static double FieldLength = 16.541;  // meter

    // --------------------------- intaker -------------------------
    public static int IntakerUpdownCANID = 40;
    public static int IntakerTurningCANID = 41;

    public static double IntakerDownPos = 1.241;
    public static double IntakerUpPos = 1.5;
    public static double IntakerRunPower = -0.5;

    // --------------------------- transfer -------------------------
    public static int TransferTurnMotor1CANID = 30;
    public static int TransferTurnMotor2CANID = 31;
    public static int TransferUpMotorCANID   = 32;

    public static double TransferTurnPower = 0.5;
    public static double TransferUpPower   = 0.5;

    // --------------------------- 驱动轮参数 -------------------------------------------
    // 左前
    public static int            DriveChannelLF                =  1;
    public static int            TurnChannelLF                 =  0;
    public static int            EncoderPortLF                 =  1; 
    public static double         TurningEncoderOffsetRadiansLF =  -0.4014; // -4.02;
    public static double         DriveGainLF                   =  1;
    public static MotorDirection TurnPowerSignLF               =  MotorDirection.FORWARD;
    public static MotorDirection TurnEncoderSignLF             =  MotorDirection.REVERSE;
    public static Translation2d m_lf = new Translation2d(0.55 / 2, 0.55 / 2); // 相对于底盘中心位置
                                                                              // 正的x表示靠前，正的y表示偏左。

    // 左后
    public static int            DriveChannelLB                =  19;
    public static int            TurnChannelLB                 =  18;
    public static int            EncoderPortLB                 =  3; 
    public static double         TurningEncoderOffsetRadiansLB = -5.409;  
    public static double         DriveGainLB                   =  1;
    public static MotorDirection TurnPowerSignLB               =  MotorDirection.FORWARD;
    public static MotorDirection TurnEncoderSignLB             =  MotorDirection.REVERSE;
    public static Translation2d m_lb = new Translation2d(-0.55 / 2, 0.55 / 2);

    // 右前
    public static int            DriveChannelRF                =  3;
    public static int            TurnChannelRF                 =  2;
    public static int            EncoderPortRF                 =  2; 
    public static double         TurningEncoderOffsetRadiansRF =  0; // -5.222;  // 2.128
    public static double         DriveGainRF                   =  1;
    public static MotorDirection TurnPowerSignRF               =  MotorDirection.FORWARD;
    public static MotorDirection TurnEncoderSignRF             =  MotorDirection.REVERSE;
    public static Translation2d m_rf = new Translation2d(0.55 / 2, -0.55 / 2);

    // 右后
    public static int            DriveChannelRB                =  17;
    public static int            TurnChannelRB                 =  16;
    public static int            EncoderPortRB                 =  0; 
    public static double         TurningEncoderOffsetRadiansRB = 2.286;// -4.881;
    public static double         DriveGainRB                   =  -1;
    public static MotorDirection TurnPowerSignRB               =  MotorDirection.REVERSE;
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

    public static String LLNameFront = "limelight-front";
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

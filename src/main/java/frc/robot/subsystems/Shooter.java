package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Shooter {
  private final TalonFX topFlywheelMotor;
  private final TalonFX bottomFlywheelMotor;
  private final TalonFX verticalAngleMotor;
  private final SparkMax horizontalAngleMotor;

  private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0);
  private final PositionVoltage verticalAngleRequest = new PositionVoltage(0);
  private final SparkClosedLoopController horizontalAngleController;

  private final RelativeEncoder horizontalEncoder;

  private double targetFlywheelVelocityRPS = 0;
  private double targetVerticalAngleDeg = 0;
  private double targetHorizontalAngleDeg = 0;

  public Shooter() {
    // 初始化三个 TalonFX + 一个 SparkMax（水平角度）
    topFlywheelMotor = new TalonFX(Constants.ShooterTopFlywheelCANID);
    bottomFlywheelMotor = new TalonFX(Constants.ShooterBottomFlywheelCANID);
    verticalAngleMotor = new TalonFX(Constants.ShooterVerticalAngleCANID);
    horizontalAngleMotor = new SparkMax(Constants.ShooterHorizontalAngleCANID, SparkLowLevel.MotorType.kBrushless);
    horizontalEncoder = horizontalAngleMotor.getEncoder();
    horizontalAngleController = horizontalAngleMotor.getClosedLoopController();

    // 飞轮配置：速度闭环
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.CurrentLimits.StatorCurrentLimit = Constants.ShooterFlywheelStatorCurrentLimit;
    flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    flywheelConfig.CurrentLimits.SupplyCurrentLimit = Constants.ShooterFlywheelSupplyCurrentLimit;
    flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    flywheelConfig.Slot0.kP = Constants.ShooterFlywheelKP;
    flywheelConfig.Slot0.kI = Constants.ShooterFlywheelKI;
    flywheelConfig.Slot0.kD = Constants.ShooterFlywheelKD;
    flywheelConfig.Slot0.kS = Constants.ShooterFlywheelKS;
    flywheelConfig.Slot0.kV = Constants.ShooterFlywheelKV;

    // 角度电机配置：位置闭环 + 重力补偿 kG
    TalonFXConfiguration angleConfig = new TalonFXConfiguration();
    angleConfig.CurrentLimits.StatorCurrentLimit = Constants.ShooterAngleStatorCurrentLimit;
    angleConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    angleConfig.CurrentLimits.SupplyCurrentLimit = Constants.ShooterAngleSupplyCurrentLimit;
    angleConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    angleConfig.Slot0.kP = Constants.ShooterAngleKP;
    angleConfig.Slot0.kI = Constants.ShooterAngleKI;
    angleConfig.Slot0.kD = Constants.ShooterAngleKD;
    angleConfig.Slot0.kS = Constants.ShooterAngleKS;
    angleConfig.Slot0.kV = Constants.ShooterAngleKV;
    angleConfig.Slot0.kG = Constants.ShooterAngleKG; // 持续向上抵消重力

    // 分别设置反转并应用（两飞轮需要不同的反转）
    flywheelConfig.MotorOutput.Inverted = Constants.ShooterTopFlywheelInverted
        ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    topFlywheelMotor.getConfigurator().apply(flywheelConfig, 0.050);

    flywheelConfig.MotorOutput.Inverted = Constants.ShooterBottomFlywheelInverted
        ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    bottomFlywheelMotor.getConfigurator().apply(flywheelConfig, 0.050);

    // 角度电机同理
    angleConfig.MotorOutput.Inverted = Constants.ShooterVerticalAngleInverted
        ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    verticalAngleMotor.getConfigurator().apply(angleConfig, 0.050);

    // 水平电机 SparkMax 配置：位置闭环（无重力补偿 kG）
    SparkMaxConfig horizontalConfig = new SparkMaxConfig();
    horizontalConfig.smartCurrentLimit((int) Constants.ShooterAngleSupplyCurrentLimit);
    horizontalConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
    horizontalConfig.inverted(Constants.ShooterHorizontalAngleInverted);

    horizontalConfig.closedLoop.pid(
        Constants.ShooterHorizontalSparkMaxKP,
        Constants.ShooterHorizontalSparkMaxKI,
        Constants.ShooterHorizontalSparkMaxKD,
        ClosedLoopSlot.kSlot0);
    horizontalConfig.closedLoop.iZone(Constants.ShooterHorizontalSparkMaxKIZone, ClosedLoopSlot.kSlot0);

    horizontalConfig.encoder.positionConversionFactor(Constants.ShooterHorizontalEncoderToDeg);

    horizontalAngleMotor.configure(horizontalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // 设置中性模式
    topFlywheelMotor.setNeutralMode(NeutralModeValue.Coast);
    bottomFlywheelMotor.setNeutralMode(NeutralModeValue.Coast);
    verticalAngleMotor.setNeutralMode(NeutralModeValue.Brake);

    // 初始化时归零
    zeroEncoders();
    verticalAngleMotor.setPosition(Constants.ShooterVerticalInitDeg / Constants.ShooterVerticalEncoderToDeg);
  }

  /** 把四个电机的编码器都重置为 0 */
  public void zeroEncoders() {
    topFlywheelMotor.setPosition(0);
    bottomFlywheelMotor.setPosition(0);
    verticalAngleMotor.setPosition(0);
    horizontalEncoder.setPosition(0);
  }

  /** 以相同速度、相反方向驱动两个飞轮（velocity 为上飞轮方向的速度，RPS） */
  public void runFlywheels(double velocityRPS) {
    targetFlywheelVelocityRPS = velocityRPS;
    topFlywheelMotor.setControl(
        flywheelVelocityRequest.withVelocity(velocityRPS));
    bottomFlywheelMotor.setControl(
        flywheelVelocityRequest.withVelocity(velocityRPS));
  }

  /** 停止两个飞轮 */
  public void stopFlywheels() {
    targetFlywheelVelocityRPS = 0;
    topFlywheelMotor.stopMotor();
    bottomFlywheelMotor.stopMotor();
  }

  /** 直接给飞轮开环占空比，调方向/测试用 */
  public void setFlywheelPower(double power) {
    topFlywheelMotor.set(power);
    bottomFlywheelMotor.set(power);
  }

  /** 设置炮台俯仰角度（度），会自动限位 */
  public void setVerticalAngle(double angleDeg) {
    targetVerticalAngleDeg = MathUtil.clamp(angleDeg, Constants.ShooterVerticalMinDeg, Constants.ShooterVerticalMaxDeg);
    double targetRotations = targetVerticalAngleDeg / Constants.ShooterVerticalEncoderToDeg;
    verticalAngleMotor.setControl(verticalAngleRequest.withPosition(targetRotations));
  }

  /** 直接给俯仰电机开环占空比，调方向/测试用 */
  public void setVerticalAnglePower(double power) {
    verticalAngleMotor.set(power);
  }

  /** 设置炮台水平角度（度），自动限位并选择最短旋转路径 */
  public void setHorizontalAngle(double angleDeg) {
    // 找到限位范围内距离当前位置最近的有效角度
    targetHorizontalAngleDeg = findNearestValidHorizontalTarget(angleDeg);
    horizontalAngleController.setSetpoint(targetHorizontalAngleDeg, ControlType.kPosition);
  }

  /** 直接给水平电机开环占空比，调方向/测试用 */
  public void setHorizontalAnglePower(double power) {
    horizontalAngleMotor.set(power);
  }

  public double getVerticalAngleDeg() {
    return verticalAngleMotor.getPosition().getValueAsDouble() * Constants.ShooterVerticalEncoderToDeg;
  }

  public double getHorizontalAngleDeg() {
    return horizontalEncoder.getPosition();
  }

  public double getTopFlywheelVelocity() {
    return topFlywheelMotor.getVelocity().getValueAsDouble();
  }

  public double getBottomFlywheelVelocity() {
    return bottomFlywheelMotor.getVelocity().getValueAsDouble();
  }

  public boolean isFlywheelAtTarget() {
    return Math.abs(targetFlywheelVelocityRPS - getTopFlywheelVelocity()) <= Constants.ShooterFlywheelVelocityToleranceRPS
        && Math.abs(targetFlywheelVelocityRPS - getBottomFlywheelVelocity()) <= Constants.ShooterFlywheelVelocityToleranceRPS;
  }

  public boolean isVerticalAtTarget() {
    return Math.abs(targetVerticalAngleDeg - getVerticalAngleDeg()) <= Constants.ShooterAngleToleranceDeg;
  }

  public boolean isHorizontalAtTarget() {
    return Math.abs(targetHorizontalAngleDeg - getHorizontalAngleDeg()) <= Constants.ShooterAngleToleranceDeg;
  }

  /** 停止所有四个电机 */
  public void stopAll() {
    topFlywheelMotor.stopMotor();
    bottomFlywheelMotor.stopMotor();
    verticalAngleMotor.stopMotor();
    horizontalAngleMotor.stopMotor();
  }

  /** 检查三个 TalonFX 和一个 SparkMax 是否都正常连接/使能 */
  public boolean areMotorsConnected() {
    return isMotorOK(topFlywheelMotor) && isMotorOK(bottomFlywheelMotor)
        && isMotorOK(verticalAngleMotor) && isSparkMaxOK(horizontalAngleMotor);
  }

  /**
   * 在水平限位范围内找到等效于 rawTargetDeg 且距离当前位置最近的目标角度。
   *
   * 搜索范围根据限位宽度自适应计算：以最接近范围中心的等效角度为起点，
   * 向两侧搜索 ceil((max-min)/720) 个 360° 步长，确保覆盖限位内的所有候选。
   *
   * @param rawTargetDeg 原始目标角度（任意值，单位：度）
   * @return 在 [ShooterHorizontalMinDeg, ShooterHorizontalMaxDeg] 内且离当前位置最近的角度
   */
  private double findNearestValidHorizontalTarget(double rawTargetDeg) {
    double current = getHorizontalAngleDeg();
    double min = Constants.ShooterHorizontalMinDeg;
    double max = Constants.ShooterHorizontalMaxDeg;
    double best = rawTargetDeg;
    double bestDist = Double.POSITIVE_INFINITY;

    // 找到使 rawTarget + k*360° 最接近范围中心的 k，作为搜索起点
    double rangeCenter = (min + max) / 2.0;
    long kCenter = Math.round((rangeCenter - rawTargetDeg) / 360.0);
    // 从 kCenter 向两侧搜索，覆盖整个限位范围的等效角度
    long searchRadius = (long) Math.ceil((max - min) / 720.0);

    for (long k = kCenter - searchRadius; k <= kCenter + searchRadius; k++) {
      double candidate = rawTargetDeg + k * 360.0;
      if (candidate >= min && candidate <= max) {
        double dist = Math.abs(candidate - current);
        if (dist < bestDist) {
          bestDist = dist;
          best = candidate;
        }
      }
    }
    best = MathUtil.clamp(best, min, max);
    return best;
  }

  private boolean isMotorOK(TalonFX motor) {
    return motor.getDeviceEnable().getStatus().isOK();
  }

  private boolean isSparkMaxOK(SparkMax motor) {
    return motor.getFirmwareVersion() != 0;
  }

  public void putData() {
    SmartDashboard.putNumber("Shooter Target Flywheel Vel", targetFlywheelVelocityRPS);
    SmartDashboard.putNumber("Shooter Top Vel", getTopFlywheelVelocity());
    SmartDashboard.putNumber("Shooter Bottom Vel", getBottomFlywheelVelocity());

    SmartDashboard.putNumber("Shooter Target Vertical Angle", targetVerticalAngleDeg);
    SmartDashboard.putNumber("Shooter Vertical Angle", getVerticalAngleDeg());
    SmartDashboard.putNumber("Shooter Target Horizontal Angle", targetHorizontalAngleDeg);
    SmartDashboard.putNumber("Shooter Horizontal Angle", getHorizontalAngleDeg());

    // 原始编码器读数，便于调试转换系数
    SmartDashboard.putNumber("Shooter Top Motor Pos", topFlywheelMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Bottom Motor Pos", bottomFlywheelMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Vertical Motor Pos", verticalAngleMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Horizontal Motor Pos", horizontalEncoder.getPosition());

    SmartDashboard.putNumber("Shooter Top Current", topFlywheelMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Bottom Current", bottomFlywheelMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Vertical Current", verticalAngleMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Horizontal Current", horizontalAngleMotor.getOutputCurrent());

    // 每个电机的单独连接状态
    SmartDashboard.putBoolean("TopFlywheelOK", isMotorOK(topFlywheelMotor));
    SmartDashboard.putBoolean("BottomFlywheelOK", isMotorOK(bottomFlywheelMotor));
    SmartDashboard.putBoolean("VerticalAngleOK", isMotorOK(verticalAngleMotor));
    SmartDashboard.putBoolean("HorizontalAngleOK", isSparkMaxOK(horizontalAngleMotor));
  }
}



/**
 * 关于“持续向上的功率抵消重力”
你是对的，不需要 MotionMagic。最合适的方式是：

角度电机用 PositionVoltage 位置闭环；
在 TalonFXConfiguration.Slot0.kG 里设置一个固定电压，它会持续输出以抵消炮台自重；
如果俯仰角度变化导致重力矩变化较大，也可以把 setVerticalAngle() 改成：

verticalAngleMotor.setControl(
    verticalAngleRequest.withPosition(targetRotations)
                        .withFeedForward(gravityCompensationVolts));
让 gravityCompensationVolts 随当前角度动态计算。
验证结果
./gradlew compileJava ✅ 编译通过
./gradlew build ❌ 失败，原因不是代码错误，而是本地 WPILib maven 缺少 windowsarm64 原生库（hal-cpp、wpimath-cpp 等）。在能联网或补齐 WPILib 2026 依赖的环境里重新跑即可。
接下来需要做的
所有带 TODO/FIXME 的占位值都要根据实车接线/机械参数替换：

CAN ID、方向系数
电流限制、PID/FF、ShooterAngleKG
编码器→角度系数、角度限位、公差
确认方向时，建议先用 setFlywheelPower、setVerticalAnglePower、setHorizontalAnglePower 给很小的开环功率，观察电机和机械运动方向，再把正确的方向填进 Constants。 */ 
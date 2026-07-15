// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {
  public static enum MotorDirection {
    FORWARD(1), REVERSE(-1);
    public int sign;
    MotorDirection(int sign) {
      this.sign = sign;
    }
  }


  private final TalonFX m_driveMotor;
  protected final TalonFX m_turningMotor;

  private final AnalogInput m_turningEncoderInput;

  private double turningMotorOffsetRadians;

  private final PIDController m_drivePIDController = new PIDController(Constants.kPModuleDriveController, 0, 0);;
  private final PIDController m_turningPIDController = new PIDController(Constants.kPModuleTurningController, 0, 0.0001);

  private double m_driveMotorGain;
  private MotorDirection turnMotorDirection;
  protected MotorDirection turnEncoderDirection;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   * @param analogEncoderPort Analog input port for the turning encoder.
   * @param turningMotorOffsetRadians Offset to add to the turning encoder reading to align with the module's zero position.
   * @param driveMotorGain Gain to apply to the drive motor output for tuning.
   * @param turnMotorDirection 决定旋转电机方向（仅±1，调整正负使向左旋为正，向右旋为负）
   * @param turnEncoderDirection 决定旋转电机的编码器的方向（仅±1，调整正负使其同样使向左正右负）
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int analogEncoderPort,
      double turningMotorOffsetRadians,
      double driveMotorGain, 
      MotorDirection turnMotorDirection, 
      MotorDirection turnEncoderDirection
    ) {

    m_driveMotor = new TalonFX(driveMotorChannel);
    m_turningMotor = new TalonFX(turningMotorChannel);

    m_driveMotor.setNeutralMode(NeutralModeValue.Brake);
    m_turningMotor.setNeutralMode(NeutralModeValue.Brake);

    this.turningMotorOffsetRadians = turningMotorOffsetRadians;

    m_driveMotorGain = driveMotorGain;

    // 旋转符号必须是±1，这个参数只影响方向，不改变大小
    this.turnMotorDirection = turnMotorDirection;
    this.turnEncoderDirection = turnEncoderDirection;

    m_turningEncoderInput = new AnalogInput(analogEncoderPort);
    
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_driveMotor.getPosition().getValueAsDouble() * Constants.kDriveEncoderDistancePerPulse, new Rotation2d(getTurningEncoderRadians()));
  }

  public double getTurningEncoderRadians(){
    double angle = (1.0 - (m_turningEncoderInput.getVoltage()/RobotController.getVoltage5V())) * 2.0 * Math.PI + turningMotorOffsetRadians - Math.PI;
    while(angle >  Math.PI) angle -= 2.0 * Math.PI;
    while(angle < -Math.PI) angle += 2.0 * Math.PI;

    return angle * turnEncoderDirection.sign;
  }

  // public double getTurningEncoderVoltage() {
  //   return m_turningEncoderInput.getVoltage();
  // }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      m_driveMotor.getVelocity().getValueAsDouble() / Constants.DriveEncoderRotationePerMetre, 
      new Rotation2d(getTurningEncoderRadians())
      );
  }

  public double getVelocity() {
    return m_driveMotor.getVelocity().getValueAsDouble();
  }

  public void stop(){
    m_driveMotor.set(0);
    m_turningMotor.set(0);
  }
  
  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState state = desiredState;

    SmartDashboard.putNumber("TargetRadians", state.angle.getRadians());
    SmartDashboard.putNumber("TurningRadians", getTurningEncoderRadians());

    // Prevent rotating module if speed is less than 0.1%. Prevents Jittering.
    // if (Math.abs(state.speedMetersPerSecond) < 0.001) {
    //     stop();
    //     return;
    // }
    
    state.optimize(new Rotation2d(getTurningEncoderRadians()));

    // Calculate the drive output from the drive PID controller.
    // Note: due to the drive PID constants being zero currently, this driveOutput will
    //       always be zero.
    final double driveOutput =
      m_drivePIDController.calculate(
        m_driveMotor.getVelocity().getValueAsDouble() / Constants.DriveEncoderRotationePerMetre, 
        state.speedMetersPerSecond);


    final double driveFeedForward = state.speedMetersPerSecond / Constants.kMaxSpeedMetersPerSecond;

    var turnOutput = 0.0D;
    
    turnOutput = m_turningPIDController.calculate(getTurningEncoderRadians(), state.angle.getRadians());

    SmartDashboard.putNumber("TargetRadians2", state.angle.getRadians());
    SmartDashboard.putNumber("TurnOutput", turnOutput);
    SmartDashboard.putNumber("DriveMotorPosition", m_driveMotor.getPosition().getValueAsDouble());


    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(
        MathUtil.clamp(
            (driveOutput + driveFeedForward) * m_driveMotorGain,
             -1.0, // min -100%
             1.0) // max +100%
    );


    m_turningMotor.set(turnMotorDirection.sign*turnOutput);
    // m_turningMotor.set(Math.copySign(turnMotorGain, turnOutput));
  }

  public StatusCode getDriveMotorStatus() {
    return m_driveMotor.getDeviceEnable().getStatus();
  }
  public StatusCode getTurnMotorStatus() {
    return m_turningMotor.getDeviceEnable().getStatus();
  }
}
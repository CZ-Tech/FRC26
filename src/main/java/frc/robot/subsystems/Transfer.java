package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * 传输子系统 —— 将地面球拨入并上传至发射器
 * - 2x SparkMax (NEO)：底部拨球滚轮
 * - 1x TalonFX：上传传送带
 */
public class Transfer {
    // 拨球滚轮电机（两个 SparkMax）
    private final SparkMax turnMotor1;
    private final SparkMax turnMotor2;
    private final RelativeEncoder turnEncoder1;

    // 上传电机（一个 TalonFX）
    private final TalonFX upMotor;

    // 当前目标功率（用于日志）
    private double targetTurnPower = 0;
    private double targetUpPower = 0;

    public Transfer() {
        // --- 拨球滚轮电机（SparkMax）---
        turnMotor1 = new SparkMax(Constants.TransferTurnMotor1CANID, SparkLowLevel.MotorType.kBrushless);
        turnMotor2 = new SparkMax(Constants.TransferTurnMotor2CANID, SparkLowLevel.MotorType.kBrushless);
        turnEncoder1 = turnMotor1.getEncoder();

        // --- 上传电机（TalonFX）---
        upMotor = new TalonFX(Constants.TransferUpMotorCANID);

        // 上传电机配置：电流限制
        TalonFXConfiguration upConfig = new TalonFXConfiguration();
        upConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        upConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        upConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        upConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        upMotor.getConfigurator().apply(upConfig, 0.050);

        // 上传电机中性模式：刹车（防止球在传送带上滚落）
        upMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    /* ==================== 拨球滚轮控制（SparkMax） ==================== */

    /**
     * 开始拨球（两个滚轮同向转动，将球向中间拨入）
     */
    public void turn() {
        targetTurnPower = Constants.TransferTurnPower;
        turnMotor1.set(Constants.TransferTurnPower);
        turnMotor2.set(Constants.TransferTurnPower);
    }

    /**
     * 反向拨球（排障/复位用）
     */
    public void turnReverse() {
        targetTurnPower = -Constants.TransferTurnPower;
        turnMotor1.set(-Constants.TransferTurnPower);
        turnMotor2.set(-Constants.TransferTurnPower);
    }

    /**
     * 停止拨球滚轮
     */
    public void stopTurn() {
        targetTurnPower = 0;
        turnMotor1.stopMotor();
        turnMotor2.stopMotor();
    }

    /**
     * 两个滚轮同时设相同开环功率（-1..1）
     */
    public void setTurnPower(double power) {
        targetTurnPower = power;
        turnMotor1.set(power);
        turnMotor2.set(power);
    }

    /**
     * 分别设每个滚轮的开环功率 —— 用于单独测试
     */
    public void setTurnPower(double power1, double power2) {
        targetTurnPower = (power1 + power2) / 2.0; // 日志用平均值
        turnMotor1.set(power1);
        turnMotor2.set(power2);
    }

    /* ==================== 上传控制（TalonFX） ==================== */

    /**
     * 向上传输（正功率）
     */
    public void up() {
        targetUpPower = Constants.TransferUpPower;
        upMotor.set(Constants.TransferUpPower);
    }

    /**
     * 反向传输（向下，排障用）
     */
    public void upReverse() {
        targetUpPower = -Constants.TransferUpPower;
        upMotor.set(-Constants.TransferUpPower);
    }

    /**
     * 停止上传电机
     */
    public void stopUp() {
        targetUpPower = 0;
        upMotor.stopMotor();
    }

    /**
     * 手动设置上传功率 —— 用于单独测试（-1..1）
     */
    public void setUpPower(double power) {
        targetUpPower = power;
        upMotor.set(power);
    }

    /* ==================== 批量控制 ==================== */

    /**
     * 停止所有电机（紧急停止）
     */
    public void stopAll() {
        targetTurnPower = 0;
        targetUpPower = 0;
        turnMotor1.stopMotor();
        turnMotor2.stopMotor();
        upMotor.stopMotor();
    }

    /* ==================== 电机健康检查 ==================== */

    /**
     * 检查电机是否都正常连接
     */
    public boolean areMotorsConnected() {
        return isTalonFXOK(upMotor);
        // SparkMax 没有简单的连接检查 API，暂时只检查 TalonFX
    }

    private boolean isTalonFXOK(TalonFX motor) {
        return motor.getDeviceEnable().getStatus().isOK();
    }

    /* ==================== SmartDashboard 日志 ==================== */

    public void putLog() {
        // 拨球滚轮
        SmartDashboard.putNumber("Transfer Turn1 Vel", turnEncoder1.getVelocity());
        SmartDashboard.putNumber("Transfer Turn1 Current", turnMotor1.getOutputCurrent());
        SmartDashboard.putNumber("Transfer Turn1 Output", turnMotor1.getAppliedOutput());
        SmartDashboard.putNumber("Transfer Turn2 Current", turnMotor2.getOutputCurrent());
        SmartDashboard.putNumber("Transfer Turn2 Output", turnMotor2.getAppliedOutput());

        // 上传电机
        SmartDashboard.putNumber("Transfer Up Pos", upMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Transfer Up Vel", upMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Transfer Up Current", upMotor.getStatorCurrent().getValueAsDouble());

        // 目标功率
        SmartDashboard.putNumber("Transfer TargetTurnPower", targetTurnPower);
        SmartDashboard.putNumber("Transfer TargetUpPower", targetUpPower);

        // 连接状态
        SmartDashboard.putBoolean("Transfer UpMotor OK", isTalonFXOK(upMotor));
    }
}

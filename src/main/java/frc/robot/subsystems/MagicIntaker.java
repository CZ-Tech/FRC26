package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;


public class MagicIntaker {
    private final TalonFX upDownMotor1;
    private final TalonFX upDownMotor2;
    private final SparkMax turningMotor1;
    private final SparkMax turningMotor2;
    private final RelativeEncoder turningEncoder1;
    private final RelativeEncoder turningEncoder2;

    // 使用 Motion Magic 的控制请求
    private final MotionMagicVoltage motionMagicRequest;

    private State currState = State.UP;
    private boolean autoPower = true;

    // 状态机
    public enum State {
        UP, DOWN
    }

    public MagicIntaker() {
        // 收放电机
        upDownMotor1 = new TalonFX(Constants.IntakerUpdownCANID);
        upDownMotor2 = new TalonFX(Constants.IntakerUpdown2CANID);

        // 滚轮电机
        turningMotor1 = new SparkMax(Constants.IntakerTurningCANID, SparkLowLevel.MotorType.kBrushless);
        turningEncoder1 = turningMotor1.getEncoder();
        turningMotor2 = new SparkMax(Constants.IntakerTurning2CANID, SparkLowLevel.MotorType.kBrushless);
        turningEncoder2 = turningMotor2.getEncoder();

        // 设置正反转方向
        turningMotor1.setInverted(Constants.IntakerTurningInverted);
        turningMotor2.setInverted(Constants.IntakerTurning2Inverted);

        /* ==================== 升降电机 (CTRE TalonFX) 配置 ==================== */
        TalonFXConfiguration fxConfigs = new TalonFXConfiguration();

        // 1. PID 参数 (用来追踪 Motion Magic 生成的曲线)
        // 注意：因为没有前馈，这里的 kP 需要稍微给够一点，以抵抗重力造成的稳态误差
        fxConfigs.Slot0.kP = 20.0;
        fxConfigs.Slot0.kI = 0.0; // 如果到位后机构往下掉，可以加一点点 kI 配合 kIZone
        fxConfigs.Slot0.kD = 0.0; // 给一点 D 可以抑制抖动
        fxConfigs.Slot0.kS = 0.1; // 克服静摩擦的最小电压
        fxConfigs.Slot0.kV = 0.12;

        // 2. Motion Magic (梯形轨迹) 参数配置
        // 这里的单位默认是 RPS (Rotations Per Second, 每秒圈数)
        fxConfigs.MotionMagic.MotionMagicCruiseVelocity = 5; // 巡航速度：机构运行时的最大速度 (需根据你的减速比调)
        fxConfigs.MotionMagic.MotionMagicAcceleration = 5;   // 加速度：到达巡航速度的快慢 (越大越猛)
        fxConfigs.MotionMagic.MotionMagicJerk = 0;            // 设为 0 就是标准梯形曲线；如果还觉得起步/刹车太猛，可以给个 100~400 (S型曲线)

        fxConfigs.MotorOutput.Inverted = Constants.IntakerUpdownInverted
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;
        upDownMotor1.getConfigurator().apply(fxConfigs);

        fxConfigs.MotorOutput.Inverted = Constants.IntakerUpdown2Inverted
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;
        upDownMotor2.getConfigurator().apply(fxConfigs);

        // 初始化控制请求
        motionMagicRequest = new MotionMagicVoltage(0);

        // 可选：初始化时将当前位置设为 UP 的坐标点
        resetEncoderToUp();
    }

    /* ==================== 升降控制逻辑 ==================== */

    public void down() {
        autoPower = true;
        currState = State.DOWN;
        // 只需要发这一条指令，TalonFX 内部会自动按梯形曲线走过去
        upDownMotor1.setControl(motionMagicRequest.withPosition(Constants.IntakerDownPos));
        upDownMotor2.setControl(motionMagicRequest.withPosition(Constants.IntakerDownPos));
    }

    public void up() {
        autoPower = true;
        currState = State.UP;
        // 同理，自动平滑上升
        upDownMotor1.setControl(motionMagicRequest.withPosition(Constants.IntakerUpPos));
        upDownMotor2.setControl(motionMagicRequest.withPosition(Constants.IntakerUpPos));
    }

    public void toggle() {
        if (currState == State.UP) {
            down();
        } else {
            up();
        }
    }

    public void manualUpDown(double power) {
        autoPower = false;
        upDownMotor1.set(power); // 直接给开环占空比/电压
        upDownMotor2.set(power);
    }

    public void resetEncoderToUp() {
        upDownMotor1.setPosition(Constants.IntakerUpPos);
        upDownMotor2.setPosition(Constants.IntakerUpPos);
    }

    /* ==================== 滚轮吸取逻辑 ==================== */

    /*
     * 开始吸取
     */
    public void intake() {
        turningMotor1.set(Constants.IntakerRunPower);
        turningMotor2.set(Constants.IntakerRunPower);
    }

    /*
     * 停止吸取
     */
    public void stopIntake() {
        turningMotor1.stopMotor();
        turningMotor2.stopMotor();
    }

    public void putLog() {
        SmartDashboard.putNumber("Intaker Position1", upDownMotor1.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intaker Position2", upDownMotor2.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intaker Target", upDownMotor1.getClosedLoopReference().getValueAsDouble());
        SmartDashboard.putString("Intaker State", currState.toString());
        SmartDashboard.putNumber("Intaker Wheel Vel1", turningEncoder1.getVelocity());
        SmartDashboard.putNumber("Intaker Wheel Vel2", turningEncoder2.getVelocity());
    }
}

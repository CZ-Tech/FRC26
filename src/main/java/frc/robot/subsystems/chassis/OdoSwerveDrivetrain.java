package frc.robot.subsystems.chassis;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class OdoSwerveDrivetrain extends SubsystemBase {
    public Rotation2d angleOffset = new Rotation2d();
    public final SwerveChassis modules;
    private ChassisSwerveOdo odo;

    public OdoSwerveDrivetrain(SwerveChassis modules, ChassisSwerveOdo odo) {
        this.modules = modules;
        this.odo = odo;
    }

    public void setNowForward() {
        angleOffset = Rotation2d.kZero.minus(odo.getHeading_wpiBlue());
    }

    /**
     * 单位m/s
     * @param axial
     * @param lateral
     * @param yaw
     */
    public void driveRobotFieldCentricLocalAlliance(double axial, double lateral, double yaw) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                axial,
                lateral,
                yaw,
                getCustomHeading());
        driveRobot(speeds);
    }

    public Rotation2d getCustomHeading() {
        SmartDashboard.putNumber("headAngle", odo.getHeading_wpiBlue().getDegrees());
        if (Robot.alliance == Alliance.Red) {
            return odo.getHeading_wpiBlue().plus(angleOffset).plus(Rotation2d.kPi);
        } else {
            return odo.getHeading_wpiBlue().plus(angleOffset);
        }
    }

    public void driveRobotFieldCentric_wpiBlue(double axial, double lateral, double yaw) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                axial,
                lateral,
                yaw,
                odo.getHeading_wpiBlue());
        driveRobot(speeds);
    }

    /**
     * 驱动底盘（以车头为x方向）
     * 
     * @param axial   功率
     * @param lateral 功率
     * @param yaw     功率
     */
    public void driveRobot(double axial, double lateral, double yaw) {
        ChassisSpeeds speeds = new ChassisSpeeds(
                axial,
                lateral,
                yaw);
        driveRobot(speeds);
    }

    public void driveRobot(ChassisSpeeds speeds) {
        // 转换为矢量轮状态
        // 此处顺序由SwerveDriveKinematics初始化时传入的Translation2d[]顺序决定
        SwerveModuleState[] moduleStates = modules.m_kinematics().toSwerveModuleStates(speeds);
        modules.lf().setDesiredState(moduleStates[0]);
        modules.rf().setDesiredState(moduleStates[1]);
        modules.lb().setDesiredState(moduleStates[2]);
        modules.rb().setDesiredState(moduleStates[3]);
    }

    public ChassisSpeeds getVelocity_robotRelative() {
        return modules.m_kinematics().toChassisSpeeds(
            modules.lf().getState(), 
            modules.rf().getState(),
            modules.lb().getState(),
            modules.rb().getState()
        );
    }

    // /**
    // * 前进到目标坐标
    // * @param t 时间（秒）
    // * @param xPos x坐标
    // * @param yPos y坐标
    // * @param yaw 目标朝向
    // */
    // public void gotoRobotFieldCentric(double t,double xPos, double yPos,double
    // yaw){
    // robot.pinpointTrajectory.addPoint(t, xPos, yPos, 0, 0, yaw);
    // }

    public void putWheelHeadingDeg() {
        SmartDashboard.putNumber("LF", Math.toDegrees(modules.lf().getTurningEncoderRadians()));
        SmartDashboard.putNumber("RF", Math.toDegrees(modules.rf().getTurningEncoderRadians()));
        SmartDashboard.putNumber("LB", Math.toDegrees(modules.lb().getTurningEncoderRadians()));
        SmartDashboard.putNumber("RB", Math.toDegrees(modules.rb().getTurningEncoderRadians()));
    }

    public boolean isMotorConnected() {
        return modules.isMotorConnected();
    }

    public void putMotorStatus() {
        modules.putMotorStatus();
    }

}

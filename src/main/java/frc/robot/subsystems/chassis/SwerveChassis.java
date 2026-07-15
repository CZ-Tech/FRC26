package frc.robot.subsystems.chassis;

import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.lib.SwerveModule;

public record SwerveChassis(
        SwerveModule lf,
        SwerveModule lb,
        SwerveModule rf,
        SwerveModule rb,
        SwerveDriveKinematics m_kinematics) {

    private static SwerveChassis instance;

    public static synchronized SwerveChassis getInstance() {
        if (instance == null) {
            instance = new SwerveChassis(
                    new SwerveModule(
                            Constants.DriveChannelLF,
                            Constants.TurnChannelLF,
                            Constants.EncoderPortLF,
                            Constants.TurningEncoderOffsetRadiansLF,
                            Constants.DriveGainLF,
                            Constants.TurnPowerSignLF,
                            Constants.TurnEncoderSignLF),
                    new SwerveModule(
                            Constants.DriveChannelLB,
                            Constants.TurnChannelLB,
                            Constants.EncoderPortLB,
                            Constants.TurningEncoderOffsetRadiansLB,
                            Constants.DriveGainLB,
                            Constants.TurnPowerSignLB,
                            Constants.TurnEncoderSignLB),
                    new SwerveModule(
                            Constants.DriveChannelRF,
                            Constants.TurnChannelRF,
                            Constants.EncoderPortRF,
                            Constants.TurningEncoderOffsetRadiansRF,
                            Constants.DriveGainRF,
                            Constants.TurnPowerSignRF,
                            Constants.TurnEncoderSignRF),
                    new SwerveModule(
                            Constants.DriveChannelRB,
                            Constants.TurnChannelRB,
                            Constants.EncoderPortRB,
                            Constants.TurningEncoderOffsetRadiansRB,
                            Constants.DriveGainRB,
                            Constants.TurnPowerSignRB,
                            Constants.TurnEncoderSignRB),
                    new SwerveDriveKinematics( // 每个轮子位置相对机器中心的偏移量
                            new Translation2d[] { // 详情参见https://docs.wpilib.org/zh-cn/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
                                    Constants.m_lf, // 此处顺序决定了后续toSwerveModuleStates(ChassisSpeeds)得到的数组顺序
                                    Constants.m_rf,
                                    Constants.m_lb,
                                    Constants.m_rb
                            }));
        }
        return instance;
    }

    /**
     * 获取各轮编码器信息
     * 
     * @return
     */
    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
                lf.getPosition(),
                rf.getPosition(),
                lb.getPosition(),
                rb.getPosition()
        };
    }

    static String[] statuStringLf = new String[] { "", "" };
    static String[] statuStringRf = new String[] { "", "" };
    static String[] statuStringLb = new String[] { "", "" };
    static String[] statuStringRb = new String[] { "", "" };

    public void putMotorStatus() {
        statuStringLf[0] = lf.getDriveMotorStatus().getName();
        statuStringLf[1] = lf.getTurnMotorStatus().getName();
        statuStringRf[0] = rf.getDriveMotorStatus().getName();
        statuStringRf[1] = rf.getTurnMotorStatus().getName();
        statuStringLb[0] = lb.getDriveMotorStatus().getName();
        statuStringLb[1] = lb.getTurnMotorStatus().getName();
        statuStringRb[0] = rb.getDriveMotorStatus().getName();
        statuStringRb[1] = rb.getTurnMotorStatus().getName();
        SmartDashboard.putString("LfSwerveDriveStatus", statuStringLf[0]);
        SmartDashboard.putString("LfSwerveTurnStatus",  statuStringLf[1]);
        SmartDashboard.putString("RfSwerveDriveStatus", statuStringRf[0]);
        SmartDashboard.putString("RfSwerveTurnStatus",  statuStringRf[1]);
        SmartDashboard.putString("LbSwerveDriveStatus", statuStringLb[0]);
        SmartDashboard.putString("LbSwerveTurnStatus",  statuStringLb[1]);
        SmartDashboard.putString("RbSwerveDriveStatus", statuStringRb[0]);
        SmartDashboard.putString("RbSwerveTurnStatus",  statuStringRb[1]);
        // 同时发布布尔值，便于 Shuffleboard 指示灯显示
        SmartDashboard.putBoolean("LfSwerveDriveOK", statuStringLf[0].equals(StatusCode.OK.getName()));
        SmartDashboard.putBoolean("LfSwerveTurnOK",  statuStringLf[1].equals(StatusCode.OK.getName()));
        SmartDashboard.putBoolean("RfSwerveDriveOK", statuStringRf[0].equals(StatusCode.OK.getName()));
        SmartDashboard.putBoolean("RfSwerveTurnOK",  statuStringRf[1].equals(StatusCode.OK.getName()));
        SmartDashboard.putBoolean("LbSwerveDriveOK", statuStringLb[0].equals(StatusCode.OK.getName()));
        SmartDashboard.putBoolean("LbSwerveTurnOK",  statuStringLb[1].equals(StatusCode.OK.getName()));
        SmartDashboard.putBoolean("RbSwerveDriveOK", statuStringRb[0].equals(StatusCode.OK.getName()));
        SmartDashboard.putBoolean("RbSwerveTurnOK",  statuStringRb[1].equals(StatusCode.OK.getName()));
    }

    public boolean isMotorConnected() {
        return lf.getDriveMotorStatus().isOK()
            && lf.getTurnMotorStatus().isOK()
            && rf.getDriveMotorStatus().isOK()
            && rf.getTurnMotorStatus().isOK()
            && lb.getDriveMotorStatus().isOK()
            && lb.getTurnMotorStatus().isOK()
            && rb.getDriveMotorStatus().isOK()
            && rb.getTurnMotorStatus().isOK();
    }

}

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.lib.LimelightHelpers;

/**
 * 实时计算安装在Shooter水平旋转环上的Limelight相对于机器人的3D位姿，
 * 并通过NetworkTables推送给Limelight用于AprilTag视觉定位。
 *
 * <h3>环的几何参数</h3>
 * 环中心位于机器人地面投影中心高523mm、中线上距中心靠后123.868mm处，
 * 半径150mm，环平面为水平面（平行于地面）。
 * Limelight沿环的位置由Shooter的horizontalEncoder角度（相对于车头方向）决定。
 *
 * <p>在 {@link Shooter#putData()} 中周期性调用 {@link #update(double)}，
 * 与 robotPeriodic 的 50Hz 周期保持一致。
 */
public class ShooterLimelightPoseCalculator {

    private final String limelightName;

    /**
     * @param limelightName Limelight 的 NetworkTables 名称，如 "limelight-front"
     */
    public ShooterLimelightPoseCalculator(String limelightName) {
        this.limelightName = limelightName;
    }

    /**
     * 根据当前水平角度计算 Limelight 在机器人坐标系中的位姿并推送给 Limelight。
     *
     * <p>计算公式（机器人坐标系 Forward=+X, Left=+Y, Up=+Z）：
     * <pre>
     *   forward = ringCenterX + ringRadius * cos(angleRad)
     *   side    = ringCenterY + ringRadius * sin(angleRad)
     *   up      = ringCenterZ（固定高度）
     * </pre>
     *
     * <p>Limelight 姿态（roll/pitch/yaw）为固定常量，不随角度变化。
     *
     * @param horizontalAngleDeg 当前水平编码器角度（度，相对车头，顺时针为正）
     */
    public void update(double horizontalAngleDeg) {
        double angleRad = Math.toRadians(
                horizontalAngleDeg + Constants.ShooterLimelightAngleOffsetDeg);

        double forward = Constants.ShooterLimelightRingCenterX
                + Constants.ShooterLimelightRingRadius * Math.cos(angleRad);
        double side = Constants.ShooterLimelightRingCenterY
                + Constants.ShooterLimelightRingRadius * Math.sin(angleRad);
        double up = Constants.ShooterLimelightRingCenterZ;

        LimelightHelpers.setCameraPose_RobotSpace(
                limelightName,
                forward, side, up,
                Constants.ShooterLimelightFixedRollDeg,
                Constants.ShooterLimelightFixedPitchDeg,
                Constants.ShooterLimelightFixedYawDeg);
    }
}

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

/**
 * 根据机器人场地坐标和已知目标坐标计算炮塔瞄准角度。
 *
 * <p>纯数学计算（不使用 Limelight 视觉反馈），在 {@link Shooter#autoAim(Pose2d, Alliance)}
 * 中每周期调用，实现"开机即跟踪"的自动瞄准。
 *
 * <h3>计算流程</h3>
 * <ol>
 *   <li>蓝方目标 = (goalFieldPosX_wpiBlue, goalFieldPosY_wpiBlue)</li>
 *   <li>红方目标 = (FieldLength - x, FieldWidth - y) 镜像</li>
 *   <li>水平角 = atan2(dy, dx) - robotHeading，归一化到 [-180°, 180°]</li>
 *   <li>垂直角 = atan2(deltaZ, distance2D)</li>
 * </ol>
 */
public class ShooterAutoAim {

    /** 蓝方目标坐标（wpiBlue 坐标系） */
    private final Translation2d blueGoal;
    /** 红方目标坐标（wpiBlue 坐标系，镜像到红方半场） */
    private final Translation2d redGoal;

    public ShooterAutoAim() {
        blueGoal = new Translation2d(
                Constants.goalFieldPosX_wpiBlue,
                Constants.goalFieldPosY_wpiBlue);
        redGoal = new Translation2d(
                Constants.FieldLength - Constants.goalFieldPosX_wpiBlue,
                Constants.FieldWidth - Constants.goalFieldPosY_wpiBlue);
    }

    /**
     * 根据机器人位姿和联盟计算目标角度。
     *
     * @param robotPose 机器人在场上的当前位姿（wpiBlue 坐标系）
     * @param alliance  当前联盟（Blue 或 Red）
     * @return 目标水平角和垂直角（度）
     */
    public TargetAngles calculate(Pose2d robotPose, Alliance alliance) {
        Translation2d target = (alliance == Alliance.Red) ? redGoal : blueGoal;

        double dx = target.getX() - robotPose.getX();
        double dy = target.getY() - robotPose.getY();

        // 水平角：机器人相对目标的方位（wpiBlue 绝对角度 → 机器人相对角度）
        double targetBearing = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeading = robotPose.getRotation().getDegrees();
        double horizontalAngle = normalizeAngle(targetBearing - robotHeading);

        // 垂直角：基于距离和高度差
        double distance2D = Math.hypot(dx, dy);
        double deltaZ = Constants.ShooterAutoAimGoalHeightMeters
                - Constants.ShooterAutoAimShooterHeightMeters;
        double verticalAngle = Math.toDegrees(Math.atan2(deltaZ, distance2D));

        return new TargetAngles(horizontalAngle, verticalAngle);
    }

    /** 归一化角度到 (-180°, 180°] */
    private static double normalizeAngle(double angle) {
        while (angle > 180.0) {
            angle -= 360.0;
        }
        while (angle <= -180.0) {
            angle += 360.0;
        }
        return angle;
    }

    /** 瞄准目标角度 */
    public record TargetAngles(double horizontalAngleDeg, double verticalAngleDeg) {}
}

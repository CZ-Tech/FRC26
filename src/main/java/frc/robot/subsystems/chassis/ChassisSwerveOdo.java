package frc.robot.subsystems.chassis;

import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.frames.TaskLoopFrame;
import frc.robot.lib.LimelightHelpers;

public class ChassisSwerveOdo {
    private AHRS gyro;
    private SwerveDrivePoseEstimator m_poseEstimator;
    private TaskLoopFrame estimateTask;
    private SwerveChassis modules;
    private boolean isReady;
    private long lastSawTagNanos;

    public ChassisSwerveOdo(SwerveChassis modules, AHRS gyroAhrs) {
        gyro = gyroAhrs;
        this.modules = modules;
        isReady = false;
        m_poseEstimator = new SwerveDrivePoseEstimator( // 初始化EKF滤波器
                modules.m_kinematics(), // SwerveDriveKinematics 对象
                gyro.getRotation2d(), // 当前角度
                new SwerveModulePosition[] {
                        modules.lf().getPosition(),
                        modules.rf().getPosition(),
                        modules.lb().getPosition(),
                        modules.rb().getPosition()
                }, // 四个模块的当前位置
                new Pose2d(0, 0, gyro.getRotation2d()), // 根据limelight确定当前坐标
                // 输入置信度（用标准差表示）
                VecBuilder.fill(Constants.SwervePosStandardDeviationFactor, Constants.SwervePosStandardDeviationFactor,
                        Constants.SwervePosStandardDeviationFactor), // 状态标准差 (x, y, theta)：数值越小越信任里程计
                VecBuilder.fill(Constants.VisionPosStandardDeviationFactor, Constants.VisionPosStandardDeviationFactor,
                        Constants.VisionPosStandardDeviationFactor) // 视觉标准差 (x, y, theta)：数值越小越信任视觉
        );

        estimateTask = new TaskLoopFrame(100, "odo") { // 启动坐标积分循环
            @Override
            public void update() {
                if (!isReady) { // 没准备好就继续准备
                    isReady = initByApriltag();
                }
                updateEstimate();
            }
        };
        estimateTask.start();

        // new TaskLoopFrame(20, "odo") {
        // @Override
        // public void update() {

        // }
        // }.start();
    }

    public boolean isReady() {
        return isReady;
    }

    private boolean initByApriltag() {
        // 1. 获取 MegaTag1 数据 (必须用 MegaTag1 来获取绝对旋转角)
        LimelightHelpers.PoseEstimate mtFront = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.LLNameFront);
        LimelightHelpers.PoseEstimate mtLeft = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.LLNameLeft);
        LimelightHelpers.PoseEstimate mtRight = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.LLNameRight);

        // 只有单个limelight看到 2 个或以上标签，才认为角度是可靠的
        boolean frontReliable = mtFront.tagCount >= Constants.leastTagStart;
        boolean leftReliable = mtLeft.tagCount >= Constants.leastTagStart;
        boolean rightReliable = mtRight.tagCount >= Constants.leastTagStart;

        SmartDashboard.putBoolean("Init_FrontReady", frontReliable);
        SmartDashboard.putBoolean("Init_LeftReady", leftReliable);
        SmartDashboard.putBoolean("Init_RightReady", rightReliable);

        // 如果三个相机都没看到足够的标签，拒绝初始化
        if (!frontReliable && !leftReliable && !rightReliable) {
            return false;
        }

        synchronized (m_poseEstimator) {
            // 找出所有可靠相机中最近的
            LimelightHelpers.PoseEstimate bestEstimate = null;
            String bestName = "";

            if (frontReliable) {
                bestEstimate = mtFront;
                bestName = "front";
            }
            if (leftReliable) {
                if (bestEstimate == null || mtLeft.avgTagDist < bestEstimate.avgTagDist) {
                    bestEstimate = mtLeft;
                    bestName = "left";
                }
            }
            if (rightReliable) {
                if (bestEstimate == null || mtRight.avgTagDist < bestEstimate.avgTagDist) {
                    bestEstimate = mtRight;
                    bestName = "right";
                }
            }

            // 用最近的可靠相机重置底盘
            m_poseEstimator.resetPosition(gyro.getRotation2d(), modules.getSwerveModulePositions(), bestEstimate.pose);

            // 用其他可靠相机进行视觉微调融合
            if (frontReliable && !bestName.equals("front")) {
                double stdDev = Constants.VisionPosStandardDeviationFactor * mtFront.avgTagDist;
                m_poseEstimator.addVisionMeasurement(mtFront.pose, mtFront.timestampSeconds, VecBuilder.fill(stdDev, stdDev, stdDev));
            }
            if (leftReliable && !bestName.equals("left")) {
                double stdDev = Constants.VisionPosStandardDeviationFactor * mtLeft.avgTagDist;
                m_poseEstimator.addVisionMeasurement(mtLeft.pose, mtLeft.timestampSeconds, VecBuilder.fill(stdDev, stdDev, stdDev));
            }
            if (rightReliable && !bestName.equals("right")) {
                double stdDev = Constants.VisionPosStandardDeviationFactor * mtRight.avgTagDist;
                m_poseEstimator.addVisionMeasurement(mtRight.pose, mtRight.timestampSeconds, VecBuilder.fill(stdDev, stdDev, stdDev));
            }
        }

        return true; // 成功完成高精度初始化
    }

    private void updateEstimate() {
        synchronized (m_poseEstimator) {
            // 使用底盘更新里程计
            m_poseEstimator.update(
                    gyro.getRotation2d(),
                    modules.getSwerveModulePositions() // 当前四个模块的编码器信息
            );

            // 为 MegaTag2 设置机器当前朝向 (利用底盘里程计更新后的偏航角，消除翻转抖动)
            double currentHeading = m_poseEstimator.getEstimatedPosition().getRotation().getDegrees();
            LimelightHelpers.SetRobotOrientation(Constants.LLNameFront, currentHeading, 0, 0, 0, 0, 0);
            LimelightHelpers.SetRobotOrientation(Constants.LLNameLeft, currentHeading, 0, 0, 0, 0, 0);
            LimelightHelpers.SetRobotOrientation(Constants.LLNameRight, currentHeading, 0, 0, 0, 0, 0);

            // 获取三个 Limelight 的 MegaTag2 视觉位置数据
            LimelightHelpers.PoseEstimate mt2Front = LimelightHelpers
                    .getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LLNameFront);
            LimelightHelpers.PoseEstimate mt2Left = LimelightHelpers
                    .getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LLNameLeft);
            LimelightHelpers.PoseEstimate mt2Right = LimelightHelpers
                    .getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LLNameRight);

            // 如果底盘角速度过大，直接放弃本周期的视觉修正
            if (Math.abs(gyro.getRate()) > 360) {
                return;
            }

            // 独立针对每个 Limelight 进行数据处理
            // 若检测到的 AprilTag 数量 >= 2，则使用 MegaTag1 并使机器方向生效；否则使用 MegaTag2
            boolean sawTagFront = false;
            if (mt2Front.tagCount >= 2) {
                sawTagFront = applyMegaTag1Vision(Constants.LLNameFront, true);
            } else {
                sawTagFront = applyMegaTag2Vision(mt2Front);
            }

            boolean sawTagLeft = false;
            if (mt2Left.tagCount >= 2) {
                sawTagLeft = applyMegaTag1Vision(Constants.LLNameLeft, true);
            } else {
                sawTagLeft = applyMegaTag2Vision(mt2Left);
            }

            boolean sawTagRight = false;
            if (mt2Right.tagCount >= 2) {
                sawTagRight = applyMegaTag1Vision(Constants.LLNameRight, true);
            } else {
                sawTagRight = applyMegaTag2Vision(mt2Right);
            }

            // 更新监控时间戳 (只要有任意一个相机看到有效标签就更新)
            if (sawTagFront || sawTagLeft || sawTagRight) {
                lastSawTagNanos = System.nanoTime();
            }
        }
    }

    /**
     * 处理单个 Limelight 的 MegaTag2 视觉数据并将其融合进 EKF。
     * 
     * @param mt2 MegaTag2 视觉位置估计数据
     * @return 是否成功应用了视觉更新
     */
    private boolean applyMegaTag2Vision(LimelightHelpers.PoseEstimate mt2) {
        if (mt2.tagCount > 0 && mt2.avgTagDist >= Float.MIN_VALUE) {
            // 随距离增大的线性标准差
            double stdDev = Constants.VisionPosStandardDeviationFactor * mt2.avgTagDist;
            m_poseEstimator.addVisionMeasurement(
                    mt2.pose,
                    mt2.timestampSeconds,
                    VecBuilder.fill(stdDev, stdDev, 9999999.0)); // megaTag2的角度数据依赖陀螺仪，故几乎无用
            return true;
        }
        return false;
    }

    /**
     * 处理单个 Limelight 的 MegaTag1 视觉数据并将其融合进 EKF。
     * 
     * @param limelightName Limelight 的名称 (如 Constants.LLNameFront)
     * @param includeRotation 是否使机器方向(旋转)生效
     * @return 是否成功应用了视觉更新
     */
    private boolean applyMegaTag1Vision(String limelightName, boolean includeRotation) {
        // 获取 MegaTag1 数据
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

        // 过滤无效数据
        if (mt1.tagCount == 0 || mt1.avgTagDist < Float.MIN_VALUE) {
            return false;
        }

        // 计算标准差(随距离线性增长)
        double stdDev = Constants.VisionPosStandardDeviationFactor * mt1.avgTagDist;
        if (mt1.tagCount >= 2) {
            // 看到 2 个及以上标签：没有歧义，角度解算极准。
            // 插入 EKF
            m_poseEstimator.addVisionMeasurement(
                    mt1.pose,
                    mt1.timestampSeconds,
                    includeRotation ? VecBuilder.fill(stdDev, stdDev, stdDev) : VecBuilder.fill(stdDev, stdDev, 9999999.0));
            return true;
        }
        return false;
    }

    public long millisFromLastTag() {
        return Math.round((System.nanoTime() - lastSawTagNanos) * 0.000001);
    }

    public void resetPos(Pose2d currPose2d) {
        synchronized (m_poseEstimator) {
            m_poseEstimator.resetPosition(
                    gyro.getRotation2d(),
                    modules.getSwerveModulePositions(),
                    currPose2d); // 重置到指定坐标
        }
    }

    public Rotation2d getHeading_wpiBlue() {
        synchronized (m_poseEstimator) {
            return m_poseEstimator.getEstimatedPosition().getRotation();
        }
    }

    public Pose2d getBotPoseEstimate_wpiBlue() {
        synchronized (m_poseEstimator) {
            return m_poseEstimator.getEstimatedPosition();
        }
    }

    double lastHeartbeatFront;
    long lastCheckMillisFront;
    boolean connectedFront = false;

    public boolean isLimelightFrontConnected() {
        if (System.currentTimeMillis() - lastCheckMillisFront <= 1000) {
            return connectedFront;
        } else {
            lastCheckMillisFront = System.currentTimeMillis();
        }
        double currHb = LimelightHelpers.getHeartbeat(Constants.LLNameFront);
        connectedFront = currHb - lastHeartbeatFront > Double.MIN_VALUE;
        lastHeartbeatFront = currHb;
        return connectedFront;
    }

    double lastHeartbeatLeft;
    long lastCheckMillisLeft;
    boolean connectedLeft = false;

    public boolean isLimelightLeftConnected() {
        if (System.currentTimeMillis() - lastCheckMillisLeft <= 1000) {
            return connectedLeft;
        } else {
            lastCheckMillisLeft = System.currentTimeMillis();
        }
        double currHb = LimelightHelpers.getHeartbeat(Constants.LLNameLeft);
        connectedLeft = currHb - lastHeartbeatLeft > Double.MIN_VALUE;
        lastHeartbeatLeft = currHb;
        return connectedLeft;
    }

    double lastHeartbeatRight;
    long lastCheckMillisRight;
    boolean connectedRight = false;

    public boolean isLimelightRightConnected() {
        if (System.currentTimeMillis() - lastCheckMillisRight <= 1000) {
            return connectedRight;
        } else {
            lastCheckMillisRight = System.currentTimeMillis();
        }
        double currHb = LimelightHelpers.getHeartbeat(Constants.LLNameRight);
        connectedRight = currHb - lastHeartbeatRight > Double.MIN_VALUE;
        lastHeartbeatRight = currHb;
        return connectedRight;
    }

}

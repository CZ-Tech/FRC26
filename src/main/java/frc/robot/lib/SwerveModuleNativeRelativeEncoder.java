package frc.robot.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class SwerveModuleNativeRelativeEncoder extends SwerveModule {
    public SwerveModuleNativeRelativeEncoder(
      int driveMotorChannel,
      int turningMotorChannel,
      int analogEncoderPort,
      double turningMotorOffsetRadians,
      double driveMotorGain, 
      MotorDirection turnMotorDirection, 
      MotorDirection turnEncoderDirection
    ) {
        super(driveMotorChannel, turningMotorChannel, analogEncoderPort, turningMotorOffsetRadians, driveMotorGain, turnMotorDirection, turnEncoderDirection);
        m_turningMotor.setPosition(0);
    }

    @Override
    public double getTurningEncoderRadians() {
        double angle = getorg();
        while (angle > Math.PI)
            angle -= 2.0 * Math.PI;
        while (angle < -Math.PI)
            angle += 2.0 * Math.PI;

        return angle * turnEncoderDirection.sign;
    }

    public double getorg() {
        double angle = m_turningMotor.getPosition().getValueAsDouble() * 2.0 * Math.PI * 0.04164 *-1;
        return angle;
    }

}

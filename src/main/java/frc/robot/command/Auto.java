package frc.robot.command;

import java.util.concurrent.atomic.AtomicBoolean;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.chassis.ChassisSwerveOdo;
import frc.robot.subsystems.chassis.OdoSwerveDrivetrain;

public class Auto {
    private SendableChooser<Command> autoChooser;
    private static volatile AtomicBoolean initialized = new AtomicBoolean();

    private Auto(ChassisSwerveOdo odo, OdoSwerveDrivetrain drivetrain) {
        // --------- 设置pathplanner参数 -----
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                () -> odo.getBotPoseEstimate_wpiBlue(), // Robot pose supplier
                odo::resetPos, // Method to reset odometry (will be called if your auto has a starting pose)
                () -> drivetrain.getVelocity_robotRelative(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> drivetrain.driveRobot(speeds), // Method that will drive the robot given ROBOT
                                                                         // RELATIVE ChassisSpeeds. Also optionally
                                                                         // outputs
                                                                         // individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                                // holonomic drive trains
                        new PIDConstants(2, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(2, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                drivetrain // Reference to drivetrain subsystem to set requirements
        );
    }

    public void prepareAutoChooser() {
        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAuto() {
        Command command = autoChooser.getSelected();
        return command;
    }

    public static class CommandsBinder {
        public CommandsBinder add(String name, Runnable operation) {
            NamedCommands.registerCommand(name, Commands.runOnce(() -> operation.run()));
            return this;
        }

        public CommandsBinder add(String name, Command command) {
            NamedCommands.registerCommand(name, command);
            return this;
        }

        public Auto prepareAuto(ChassisSwerveOdo odo, OdoSwerveDrivetrain drivetrain) {
            synchronized (initialized) {
                if (initialized.get()) {
                    throw new IllegalStateException("Auto object can't be duplicated");
                } else {
                    initialized.set(true);
                    return new Auto(odo, drivetrain);
                }
            }
        }
    }

    public static CommandsBinder bindOperations() {
        return new CommandsBinder();
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Math stuff
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

// Constants
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

// Subsystems
import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Vision;
// WpiLib2 stuff
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// Java utils
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    // private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final Vision vision = new Vision();

    // The driver's controller
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        /* */
        m_robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new RunCommand(() -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(),
                    OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(),
                    OIConstants.kDriveDeadband),
                /* *
                -MathUtil.applyDeadband(vision.safeGetTagYaw(25),
                    OIConstants.kDriveDeadband),
                // */

                /* */
                -MathUtil.applyDeadband(m_driverController.a().getAsBoolean()
                ? vision.getTag(25).getYaw() / 180
                : m_driverController.getRightX(),
                OIConstants.kDriveDeadband),
                // */

                /* *
                -MathUtil.applyDeadband(m_driverController.getRightX(),
                    OIConstants.kDriveDeadband),
                // */
                true),
            m_robotDrive, vision));
        // */
    }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {
    m_driverController.rightBumper().whileTrue(new InstantCommand(() -> m_robotDrive.setX()));
    // m_driverController.rightTrigger().whileTrue(m_intake.runIntakeCommand());
    // m_driverController.leftTrigger().whileTrue(m_intake.runExtakeCommand());
    m_driverController.start().onTrue(new InstantCommand(() -> m_robotDrive.resetOdometry(vision.getPose2d())));
  }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                config);

        var thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                exampleTrajectory,
                m_robotDrive::getPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                m_robotDrive::setModuleStates,
                m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }
}

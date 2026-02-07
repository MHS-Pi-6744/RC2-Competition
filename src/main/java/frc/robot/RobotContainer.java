// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

// Math stuff
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// WpiLib2 stuff
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants.BlueAlliance;
import frc.robot.Constants.AutoConstants.RedAlliance;
// Constants
import frc.robot.Constants.OIConstants;
// Subsystems
import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Vision;

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
    private final Vision vision = new Vision(m_robotDrive::addVisionMeasurement);

    private final SendableChooser<Command> autoChooser;

    // The driver's controller
    public CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

    Command pathfindLeftClimbBlue = AutoBuilder.pathfindToPose(
            BlueAlliance.kLeftClimb,
            AutoConstants.kConstraints,
            0.0 // Goal end velocity in meters/sec
    );

    Command pathfindLeftClimbRed = AutoBuilder.pathfindToPose(
            RedAlliance.kLeftClimb,
            AutoConstants.kConstraints,
            0.0 // Goal end velocity in meters/sec
    );

    public boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent())
            return alliance.get() == DriverStation.Alliance.Red;
        return false;
    }

    Trigger onBlueAlliance = new Trigger(() -> !isRedAlliance());
    Trigger onRedAlliance = new Trigger(() -> isRedAlliance());

    public enum DrivingMode {
        kNormal,
        kTagAssisted
    }

    private DrivingMode drivingMode;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        drivingMode = DrivingMode.kNormal;

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);

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
                        -MathUtil.applyDeadband(getDriveRot(),
                                OIConstants.kDriveDeadband),
                        true),
                        m_robotDrive, vision));
        // */
    }

    private void driveTagAssisted() {
        m_driverController.setRumble(RumbleType.kBothRumble, 0.5);
        drivingMode = DrivingMode.kTagAssisted;
    }

    private void driveNormal() {
        m_driverController.setRumble(RumbleType.kBothRumble, 0);
        drivingMode = DrivingMode.kNormal;
    }

    /**
     * Gets the rotation of the robot based on the driving mode
     * 
     * @return the double to pass to {@link DriveSubsystem#drive()} as rot
     */
    private double getDriveRot() {
        switch (drivingMode) {
        case kNormal:
            return m_driverController.getRightX();
        case kTagAssisted:
            return vision.getTag(25).getYaw() / 180;
        default:
            return m_driverController.getRightX();
        }
    }

    /**
     * Use this method to define your button->command mappings.
     */
    private void configureButtonBindings() {
        m_driverController.rightBumper()
                .whileTrue(new InstantCommand(() -> m_robotDrive.setX()));
        // m_driverController.rightTrigger().whileTrue(m_intake.runIntakeCommand());
        // m_driverController.leftTrigger().whileTrue(m_intake.runExtakeCommand());
        m_driverController.start()
                .onTrue(new InstantCommand(() -> m_robotDrive.resetPose(vision.getPose2d())));
        m_driverController.b().and(onBlueAlliance)
                .onTrue(pathfindLeftClimbBlue);
        m_driverController.b().and(onRedAlliance)
                .onTrue(pathfindLeftClimbRed);
        m_driverController.a()
                .onTrue(new InstantCommand(() -> driveTagAssisted()))
                .onFalse(new InstantCommand(() -> driveNormal()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// WpiLib2 stuff
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs.Default;
import frc.robot.Configs.IntakeConfigs;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants.BlueAlliance;
import frc.robot.Constants.AutoConstants.RedAlliance;
import frc.robot.Constants.IntakeSubsystemConstants.PivotSetPoints;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterSubsystemConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.canIDs;
import frc.robot.commands.ApproachFuelCommand;
// Subsystems
import frc.robot.motor_ctl.MotorController;
// Constants
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ObjectDetectionSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
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
  private final PivotSubsystem m_pivot = new PivotSubsystem();
  private final MotorController m_intake =
      new MotorController(canIDs.kIntakeMotorCanId, IntakeConfigs.intakeConfig);
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final MotorController m_feeder =
      new MotorController(canIDs.kFeederMotorCanId, Default.Config.inverted(false));
  private final MotorController m_sucker =
      new MotorController(ShooterSubsystemConstants.kSuckerCanId, Default.Config.inverted(true));

  private final SendableChooser<Command> autoChooser;

  private final ObjectDetectionSubsystem objectDetection =
      new ObjectDetectionSubsystem("Object_CAM");

  // The driver's controller
  public CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  public CommandXboxController m_copilotController =
      new CommandXboxController(OIConstants.kCopilotControllerPort);

  Command pathfindLeftClimbBlue =
      AutoBuilder.pathfindToPose(
          BlueAlliance.kLeftClimb,
          AutoConstants.kConstraints,
          0.0 // Goal end velocity in meters/sec
          );

  Command pathfindLeftClimbRed =
      AutoBuilder.pathfindToPose(
          RedAlliance.kLeftClimb, AutoConstants.kConstraints, 0.0 // Goal end velocity in meters/sec
          );

  public boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) return alliance.get() == DriverStation.Alliance.Red;
    return false;
  }

  Trigger onBlueAlliance = new Trigger(() -> !isRedAlliance());
  Trigger onRedAlliance = new Trigger(() -> isRedAlliance());

  public enum DrivingMode {
    kNormal,
    kTagAssisted
  }

  private DrivingMode drivingMode;

  private Command m_feeder_run =
      new ParallelCommandGroup(m_sucker.setSpeed(0.5), m_feeder.setSpeed(0.5));
  private Command m_feeder_stop =
      new ParallelCommandGroup(m_sucker.stopMotor(), m_feeder.stopMotor());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    NamedCommands.registerCommand("Flywheel Go", m_shooter.runRPM(2750));
    NamedCommands.registerCommand("Flywheel Stop", m_shooter.stopFlywheel());
    NamedCommands.registerCommand("Feeder Go", m_feeder_run);
    NamedCommands.registerCommand("Feeder Stop", m_feeder_stop);
    NamedCommands.registerCommand(
        "Pivot Down", m_pivot.setTargetPosition(PivotSetPoints.kEndPosition));
    NamedCommands.registerCommand(
        "Pivot Up", m_pivot.setTargetPosition(PivotSetPoints.kStartPosition));
    // NamedCommands.registerCommand("Pivot Dump", m_feeder_stop);
    NamedCommands.registerCommand("Intake Forwards", m_intake.runMotor(1));
    NamedCommands.registerCommand("Intake Backwards", m_intake.runMotor(-1));
    NamedCommands.registerCommand("Intake Stop", m_intake.stopMotor());

    // Configure the button bindings
    configureButtonBindings();

    drivingMode = DrivingMode.kNormal;

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("Pathfind Left Climb Red", pathfindLeftClimbRed);

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure default commands
    /* */
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        getDriveRot(),
                        drivingMode == DrivingMode.kTagAssisted ? 0.0 : OIConstants.kDriveDeadband),
                    true),
            m_robotDrive,
            vision));
    // */

    // If logging only to DataLog
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
        var robot_pose = m_robotDrive.getPose();
        var target_pose =
            isRedAlliance() ? VisionConstants.kRedHubCenter : VisionConstants.kBluHubCenter;
        var a = target_pose.getX() - robot_pose.getX();
        var o = target_pose.getY() - robot_pose.getY();
        if (a == 0) return 0;
        var target_angle = Math.atan2(o, a);
        var diff = Units.radiansToDegrees(target_angle) - robot_pose.getRotation().getDegrees();
        return -diff / 180;
      default:
        return m_driverController.getRightX();
    }
  }

  /** Use this method to define your button -> command mappings. */
  private void configureButtonBindings() {
    // m_driverController.rightBumper().whileTrue(new InstantCommand(() ->
    // m_robotDrive.setX()));
    // m_driverController.rightTrigger().whileTrue(m_intake.runIntakeCommand());
    // m_driverController.leftTrigger().whileTrue(m_intake.runExtakeCommand());
    // m_driverController.start()
    // .onTrue(new InstantCommand(() ->
    // m_robotDrive.resetPose(vision.getPose2d())));
    m_driverController
        .a()
        .onTrue(new InstantCommand(() -> driveTagAssisted()))
        .onFalse(new InstantCommand(() -> driveNormal()));
    m_driverController.povUp().onTrue(m_shooter.incrementSetpoint(250));
    m_driverController.povDown().onTrue(m_shooter.incrementSetpoint(-250));
    m_driverController.povRight().onTrue(m_shooter.incrementSetpoint(10));
    m_driverController.povLeft().onTrue(m_shooter.incrementSetpoint(-10));
    m_driverController.rightBumper().onTrue(m_shooter.runAtSet()).onFalse(m_shooter.stopFlywheel());
    m_driverController.x().onTrue(m_feeder_run).onFalse(m_feeder_stop);
    m_driverController.start().onTrue(m_robotDrive.resetGyro());
    m_copilotController
        .rightBumper()
        .whileTrue(m_intake.runMotor(1))
        .whileFalse(m_intake.stopMotor());
    m_copilotController
        .leftBumper()
        .whileTrue(m_intake.runMotor(-1))
        .whileFalse(m_intake.stopMotor());
    m_copilotController.y().onTrue(m_pivot.setTargetPosition(PivotSetPoints.kStartPosition));
    m_copilotController.b().onTrue(m_pivot.setTargetPosition(PivotSetPoints.kMiddlePosition));
    m_copilotController.a().onTrue(m_pivot.setTargetPosition(PivotSetPoints.kEndPosition));
    m_copilotController.b().onTrue(m_pivot.setTargetPosition(PivotSetPoints.kStartPosition));
    m_driverController
        .leftTrigger()
        .whileTrue(new ApproachFuelCommand(m_robotDrive, objectDetection));
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

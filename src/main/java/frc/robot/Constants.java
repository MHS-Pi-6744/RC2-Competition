// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
// Math stuff
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class canIDs {

    /**
     * @apiNote SPARKmax - The competition robot will have 2 motors - conveyer and feeder to shooter
     * @apiNote This is the Feeder Motor Can ID
     */
    public static final int kFeederMotorCanId = 15;

    /**
     * @apiNote SPARKmax -
     * @apiNote This is the Intake Motor Can ID
     */
    public static final int kIntakeMotorCanId = 13;

    /**
     * @apiNote SPARKmax -
     * @apiNote This is the Pivot Motor of Intake Can ID
     */
    public static final int kPivotMotorCanId = 14;

    /**
     * @apiNote SPARKmax -
     * @apiNote This is the Climber Motor Can ID
     */
    public static final int kClimbMotorCanId = 15;

    // Others? PHD, RoboRio?
  }

  public static final class IntakeSubsystemConstants {

    public static final class IntakeSetpoints {
      /**
       * @apiNote The Command for setting the motor speed
       */
      public static final double kIntake = 1; // Intake speed Units are percentage
    }

    public static final class PivotSetPoints {
      public static final double kStartPosition = 5; // to stay away from zero encoder reading

      /**
       * @apiNote DEGREES
       */
      public static final double kEndPosition = 120;

      public static final int kCurrentLimit = 40;

      public static final double kZeroOffest = 0.0; // units? For stationary testbed motor

      public static final double kPositionConversionFactor =
          360 / 41; // For stationary test bed motor in deg
      public static final double kVelocityConversionFactor = 360 / 41; // This is deg/sec

      public static final IdleMode kIdleMode = IdleMode.kCoast;

      public static final double kMaxVelocity = 9000; // degrees per min
      public static final double kMaxAcceleration = 3600; // Units deg/min/sec
      public static final double kPositionTolerance = 90; // Units deg

      // PID gains    ======== Will need to be tuned when operating on the climber     -Sr
      public static final double kP = 0.10000000;
      public static final double kI = 0.00000000;
      public static final double kD = 0.00000000;

      // MAYBE LATER!
      // The pivot is expected to have hard stops
      // public static final double kFwdSoftLimit = 125;
      // public static final double kRevSoftLimit = 25;
    }
  }

  public static final class ShooterSubsystemConstants {
    // SPARKmax CAN ID (Right)
    // public static final int kFlywheelFollowerMotorCanId = 16;  // SPARKmax CAN ID (Left)

    public static final int kCenterCanId = 18;
    public static final int kRightCanId = 19;
    public static final int kLeftCanId = 17;

    public static final int kSuckerCanId = 16;

    public static final class FeederSetpoints {
      public static final double kFeed = 0.15;
    }

    // Check these units - it looks to me like ShooterSubsystem is controlling in RPM???  Sr
    public static final class FlywheelSetpoints {

      /**
       * @apiNote This controls how much the shooter will spin
       * @apiNote PERCENTAGE
       */
      public static final double kShootPercent = 50;

      public static final double kVelocityTolerance = 100;
    }
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 3; // 4.8;
    public static final double kMaxAccelerationMetersPerSecondSq = 3; // 4.8;
    public static final double kMaxAngularSpeed =
        1.5 * Math.PI; // 2 * Math.PI; // radians per second
    public static final double kMaxAngularAcceleration =
        1.5 * Math.PI; // 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(28);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(28);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontRightDrivingCanId = 5;
    public static final int kFrontLeftDrivingCanId = 6;
    public static final int kRearRightDrivingCanId = 7;
    public static final int kRearLeftDrivingCanId = 8;

    public static final int kFrontRightTurningCanId = 9;
    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearRightTurningCanId = 11;
    public static final int kRearLeftTurningCanId = 12;

    public static final int kGyroCanId = 2;

    public static final boolean kGyroReversed = false;

    public static final PIDConstants kTranslationPID = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants kRotationPID = new PIDConstants(5.0, 0.0, 0.0);

    public static final Config kSysIDConfig = new Config(null, null, null, null);
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 12;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCopilotControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final PathConstraints kConstraints =
        new PathConstraints(
            DriveConstants.kMaxSpeedMetersPerSecond,
            DriveConstants.kMaxAccelerationMetersPerSecondSq,
            DriveConstants.kMaxAngularSpeed,
            DriveConstants.kMaxAngularAcceleration);

    public static final class BlueAlliance {
      // [0.9781092627094603, 4.651867655668605, 0.23733570893866954]
      public static final Pose2d kLeftClimb = new Pose2d(1.00, 4.65, Rotation2d.fromDegrees(0));
      //
      public static final Pose2d kRightClimb = new Pose2d(1, 2.5, Rotation2d.fromDegrees(180));
    }

    public static final class RedAlliance {
      public static final Pose2d kLeftClimb = new Pose2d(15.5, 3.5, Rotation2d.fromDegrees(0));
      public static final Pose2d kRightClimb = new Pose2d(15.3, 5.5, Rotation2d.fromDegrees(180));
    }
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class VisionConstants {
    public static final String kCameraName = "Cam2";

    /** Where is the camera mounted relative to robot center? */
    public static final Transform3d kRobotToCam =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(6), Units.inchesToMeters(5.5), Units.inchesToMeters(4.75)),
            new Rotation3d(0, 0, 0));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    /** Placeholder target with almost all fields maxxed out */
    public static final PhotonTrackedTarget kMaxTarget =
        new PhotonTrackedTarget(
            Double.MAX_VALUE,
            Double.MAX_VALUE,
            Double.MAX_VALUE,
            Double.MAX_VALUE,
            -1,
            Integer.MAX_VALUE,
            Float.MAX_VALUE,
            new Transform3d(
                Double.MAX_VALUE,
                Double.MAX_VALUE,
                Double.MAX_VALUE,
                new Rotation3d(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE)),
            new Transform3d(
                Double.MAX_VALUE,
                Double.MAX_VALUE,
                Double.MAX_VALUE,
                new Rotation3d(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE)),
            Double.MAX_VALUE,
            List.of(new TargetCorner()),
            List.of(new TargetCorner()));

    /** Placeholder target with almost all fields zeroed out */
    public static final PhotonTrackedTarget kEmptyTarget =
        new PhotonTrackedTarget(
            0,
            0,
            0,
            0,
            -1,
            0,
            0,
            new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)),
            new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)),
            0,
            List.of(new TargetCorner()),
            List.of(new TargetCorner()));

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  public static final class ClimbSubsystemConstants {

    public static final class ClimbSetPoints {
      public static final double kStartPosition = 0;
      public static final double kEndPosition = 90;

      public static final int kStartingPosition = 0;
      public static final int kCurrentLimit = 50;

      public static final double kZeroOffest = 0.420;

      public static final double kPositionConversionFactor = 360 / 41; // Deg/sec
      public static final double kVelocityConversionFactor = 360 / 41; // Deg/sec

      /**
       * Sets the Idle mode of the motors.
       *
       * @apiNote This should remain as {@link IdleMode#kBrake} unless you want to manually rotate
       *     the motors
       */
      public static final IdleMode kIdleMode = IdleMode.kBrake;

      // Deprecated, but still nice:
      // In Desmos,
      // y=0.37037x+19.5\left\{0<\ x<27\right\}
      // y=0.6842x+11.0265\left\{27<x<65\right\}
      // is the approximate position curve for the elevator
      // where x is in Degrees of PCF1 and y is in Inches
      // Measured from the top of the shooter

      public static final double kMaxVelocity = 3072;
      public static final double kMaxAcceleration = 1536;

      public static final double kP = 0.50000000;
      public static final double kI = 0.00000000;
      public static final double kD = 0.00000000;

      /**
       * The soft limit for the elevator going forward.
       *
       * @apiNote This soft limit should NEVER go above 24
       */
      public static final double kFwdSoftLimit = 23;

      /**
       * The soft limit for the elevator going backward.
       *
       * @apiNote This soft limit should NEVER go below 1
       */
      public static final double kRevSoftLimit = -23;

      /**
       * The allowed tolerance for the elevator
       *
       * @apiNote This value is in inches
       * @apiNote This really shouldn't ever go above an inch.
       */
      public static final double kPositionTolerance = 0.75;

      public static final double kStageLoad = 7.86;
      public static final double kStageL1 = 6.61;
      public static final double kStageL2 = 13.65;
      public static final double kStageL3 = 22;
      public static final double kStageAlgae = 6.61;
    }
  }
}
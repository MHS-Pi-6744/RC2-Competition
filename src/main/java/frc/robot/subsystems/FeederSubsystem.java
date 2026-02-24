// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
//import com.revrobotics.spark.SparkBase.ControlType;
//import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

//import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.Constants.ShooterSubsystemConstants.FeederSetpoints;
//import frc.robot.Constants.ShooterSubsystemConstants.FlywheelSetpoints;
//import frc.robot.Constants.ShooterSubsystemConstants;
import frc.robot.Constants.canIDs;
//import frc.robot.subsystems.ShooterSubsystem;

public class FeederSubsystem extends SubsystemBase {
  
  // Initialize flywheel SPARKs. We will use MAXMotion velocity control for the flywheel, so we also need to
  // initialize the closed loop controllers and encoders.

  /*private SparkMax flywheelFollowerMotor =
      new SparkMax(ShooterSubsystemConstants.kFlywheelFollowerMotorCanId, MotorType.kBrushless);
*/
  // Initialize feeder SPARK. We will use open loop control for this so we don't need a closed loop
  // controller like above.

  private SparkMax feederMotor =
      new SparkMax(canIDs.kFeederMotorBeltCanId, MotorType.kBrushless);

  private SparkMax feederMotor2 = 
      new SparkMax(canIDs.kFeederMotorWheelCanId, MotorType.kBrushless);

  // Member variables for subsystem state management

  /** Creates a new ShooterSubsystem. */
  public FeederSubsystem() {
    /*
     * Apply the appropriate configurations to the SPARKs. 
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
     feederMotor.configure(
        Configs.ShooterSubsystem.feederConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  
    feederMotor2.configure(
      Configs.ShooterSubsystem.feederConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  }
  /** Set the feeder motor power in the range of [-1, 1]. */
  private void setFeederPower(double power) {
        feederMotor.set(power);
        feederMotor2.set(power);
  }

  /**
   * Command to run the feeder and flywheel motors. When the command is interrupted, e.g. the button is released,
   * the motors will stop.
   */
  public Command runFeederCommand() {
      return this.startEnd(
          () -> {
            this.setFeederPower(FeederSetpoints.kFeed);
            }, () -> {
            this.setFeederPower(0.0);
            }).withName("Feeding");
  }

  public Command runFeederReverseCommand() {
    return this.startEnd(
    () -> {
      this.setFeederPower(FeederSetpoints.kFeed*-1);
    }, () -> {
      this.setFeederPower(0.0);
    }).withName("Reverse Feeding");
  }

  /**
   * Meta-command to operate the shooter. The Flywheel starts spinning up and when it reaches
   * the desired speed it starts the Feeder.
   */
  @Override
  public void periodic() {
      // Display subsystem values
      SmartDashboard.putNumber("Shooter | Feeder | Applied Output", feederMotor.getAppliedOutput());
  }
}

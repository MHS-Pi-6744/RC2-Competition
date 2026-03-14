
package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeSubsystemConstants.PivotSetPoints;
import frc.robot.Constants.canIDs;

public class ClimberSubsystem extends SubsystemBase {

  private SparkMax m_climb = new SparkMax(canIDs.kPivotMotorCanId, MotorType.kBrushless);

  private RelativeEncoder re_climb;

  private SparkClosedLoopController p_climb;

  public ClimberSubsystem() {
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
    m_climb.configure(
        Configs.ClimberConfigs.climbConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    p_climb = m_climb.getClosedLoopController();

    re_climb = m_climb.getEncoder();

    setTargetPosition(
        PivotSetPoints.kStartPosition); // set target position to start position and go there
  }

  public Command setTargetPosition(double setpos) {
    return run(
        () ->
            p_climb.setSetpoint(
                setpos, ControlType.kMAXMotionPositionControl) // USING PID POSITION CONTROL
        );
  }

  public Command runMotor(double speed) {
    return run(
      () -> m_climb.set(speed)
    );
  }

  public Command stopMotor() {
    return runMotor(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climb/" + "Output", m_climb.getAppliedOutput());
    SmartDashboard.putNumber("Climb/" + "Current", m_climb.getOutputCurrent());
    SmartDashboard.putNumber("Climb/" + "Relative/" + "Position", re_climb.getPosition());
    // SmartDashboard.putNumber("Pivot/"+"Absolute/"+"Position", );
    SmartDashboard.putNumber("Climb/" + "Relative/" + "Velocity", re_climb.getVelocity());
    // SmartDashboard.putNumber("Pivot/"+"Absolute/"+"Velocity", );
  }
}



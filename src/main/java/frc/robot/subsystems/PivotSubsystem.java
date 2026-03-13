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

public class PivotSubsystem extends SubsystemBase {

  private SparkMax m_pivotMotor = new SparkMax(canIDs.kPivotMotorCanId, MotorType.kBrushless);

  private RelativeEncoder re_pivotMotor;

  private SparkClosedLoopController p_pivotMotor;

  public PivotSubsystem() {
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
    m_pivotMotor.configure(
        Configs.IntakeConfigs.pivotConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    p_pivotMotor = m_pivotMotor.getClosedLoopController();

    re_pivotMotor = m_pivotMotor.getEncoder();

    setTargetPosition(
        PivotSetPoints.kStartPosition); // set target position to start position and go there
  }

  public void slowMoveBack() {
    re_pivotMotor.setPosition(-.5);
  }

  public Command setTargetPosition(double setpos) {
    return run(
        () ->
            p_pivotMotor.setSetpoint(
                setpos, ControlType.kMAXMotionPositionControl) // USING PID POSITION CONTROL
        );
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot/" + "Output", m_pivotMotor.getAppliedOutput());
    SmartDashboard.putNumber("Pivot/" + "Current", m_pivotMotor.getOutputCurrent());
    SmartDashboard.putNumber("Pivot/" + "Relative/" + "Position", re_pivotMotor.getPosition());
    // SmartDashboard.putNumber("Pivot/"+"Absolute/"+"Position", );
    SmartDashboard.putNumber("Pivot/" + "Relative/" + "Velocity", re_pivotMotor.getVelocity());
    // SmartDashboard.putNumber("Pivot/"+"Absolute/"+"Velocity", );
  }
}

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
import frc.robot.Constants.IntakeSubsystemConstants.IntakeSetpoints;
import frc.robot.Constants.IntakeSubsystemConstants.PivotSetPoints;
import frc.robot.Constants.canIDs;

public class IntakeSubsystem extends SubsystemBase {

  // Initialize Intake and Pivot SparkMax
  private SparkMax m_intakeMotor = new SparkMax(canIDs.kIntakeMotorCanId, MotorType.kBrushless);

  private SparkMax m_pivotMotor = new SparkMax(canIDs.kPivotMotorCanId, MotorType.kBrushless);

  private RelativeEncoder re_pivotMotor;

  private SparkClosedLoopController p_pivotMotor;

  private double m_setpoint;

  public IntakeSubsystem() {
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
    m_intakeMotor.configure(
        Configs.IntakeConfigs.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_pivotMotor.configure(
        Configs.IntakeConfigs.pivotConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    p_pivotMotor = m_pivotMotor.getClosedLoopController();

    re_pivotMotor = m_pivotMotor.getEncoder();

    setTargetPosition(
        PivotSetPoints.kStartPosition); // set target position to start position and go there

    System.out.println("---> IntakeSubsystem initialized");
  }

  public void slowMoveBack() {
    re_pivotMotor.setPosition(-.5);
  }

  // public boolean atTargetPoint() {
  // return Math.abs(distancePivotAbsAndSetPoint()) <
  // PivotSetPoints.kPositionTolerance;
  // }

  // public double distancePivotAbsAndSetPoint(){
  // return re_pivotMotor.getPosition() - m_setpoint;
  // }

  public void setTargetPosition(double setpos) {
    m_setpoint = setpos;
    moveToSetPoint();
  }

  public void moveToSetPoint() {
    p_pivotMotor.setSetpoint(
        m_setpoint, ControlType.kMAXMotionPositionControl); // USING PID POSITION CONTROL
  }

  /**
   * {@link Command} to run the intake motor power {@link Command}. When the {@link Command} is
   * interrupted, e.g. the button is released, the motors will stop
   *
   * @author Pubert
   */
  public Command runIntakeCommand() {
    return this.startEnd(
            () -> m_intakeMotor.set(IntakeSetpoints.kIntake), // Make it constantly move fwd
            () -> m_intakeMotor.set(0) // Make it stop
            )
        .withName("Intaking");
  }

  /**
   * {@link Command} to run the intake motor power {@link Command}. When the {@link Command} is
   * interrupted, e.g. the button is released, the motors will stop
   *
   * @author Pubert
   */
  public Command runExtakeCommand() {
    return this.startEnd(
            () -> m_intakeMotor.set(IntakeSetpoints.kIntake * -1), // Make it constantly move bwd
            () -> m_intakeMotor.set(0) // Make it stop
            )
        .withName("Extaking");
  }

  /**
   * {@link Command} to move the Pivot Motor forward.
   *
   * @author Pubert
   */
  public Command runForwardPivot() {
    return this.run(
            () ->
                setTargetPosition(
                    PivotSetPoints.kEndPosition) // Snap to the end position with trapazoidal
            // movement
            )
        .withName("Moving Pivot Forward");
  }

  /**
   * {@link Command} to move the Pivot Motor backward.
   *
   * @author Pubert
   */
  public Command runBackwardPivot() {
    return this.run(
            () ->
                setTargetPosition(
                    PivotSetPoints.kStartPosition) // Snap to the starting position with the
            // trapazoidal movement
            )
        .withName("Moving Pivot Backward");
  }

  /**
   * {@link Command} to calibrate the Pivot Motor
   *
   * @author Pubert
   */
  public Command calPivotMotor() {
    return this.run(
            () -> slowMoveBack() // Have the Pivot Motor slowly move back so that it can be at the
            // starting
            // position without knowing where it really is.
            )
        .withName("Slowly move pivot back");
  }

  @Override
  public void periodic() {
    // Display subsystem values
    // SmartDashboard.putNumber("Intake | Intake | Applied Output",
    // m_intakeMotor.getAppliedOutput());
    SmartDashboard.putNumber("Pivot | Pivot | Applied Output", m_pivotMotor.getAppliedOutput());

    SmartDashboard.putNumber("Pivot relative pos", re_pivotMotor.getPosition());
    SmartDashboard.putNumber("Pivot Current", m_pivotMotor.getOutputCurrent());
    SmartDashboard.putNumber("Setpoint value", m_setpoint);
    SmartDashboard.putNumber("Velocity", re_pivotMotor.getVelocity());
  }
}

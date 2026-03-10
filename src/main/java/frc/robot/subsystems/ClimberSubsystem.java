package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
// import frc.robot.Constants.ClimbSubsystemConstants;
import frc.robot.Constants.ClimbSubsystemConstants;

public class ClimberSubsystem extends SubsystemBase {
  // Initialize intake Spark. We will use open loop control for this

  private SparkMax m_climbMotor =
      new SparkMax(ClimbSubsystemConstants.kClimbMotorCanId, MotorType.kBrushless);

  private SparkAbsoluteEncoder ae_climbMotor;
  private RelativeEncoder re_climbMotor;

  private SparkClosedLoopController p_climbMotor;

  public static LaserCan lc;

  private double m_setpoint;

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

    m_climbMotor.configure(
        Configs.ClimberSubsystem.climbConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_setpoint = ClimbSubsystemConstants.ClimbSetPoints.kStartPosition;

    p_climbMotor = m_climbMotor.getClosedLoopController();

    re_climbMotor = m_climbMotor.getEncoder();
    ae_climbMotor = m_climbMotor.getAbsoluteEncoder();

    re_climbMotor.setPosition(ae_climbMotor.getPosition());

    lc = new LaserCan(0);

    // re_pivotMotor.setPosition(0);

    setit();

    System.out.println("---> IntakeSubsystem initialized");
  }

  public boolean atTargetPoint() {
    return Math.abs(re_climbMotor.getPosition() - m_setpoint)
        < ClimbSubsystemConstants.ClimbSetPoints.kPositionTolerance;
  }

  public void setTargetPosition(double setpos) {
    m_setpoint = setpos;
    moveToSetPoint();
  }

  public void setit() {
    re_climbMotor.setPosition(ae_climbMotor.getPosition());
  }

  public void moveToSetPoint() {
    p_climbMotor.setSetpoint(m_setpoint, ControlType.kMAXMotionPositionControl);
  }

  /**
   * {@link Command} to run the intake motor power {@link Command}. When the {@link Command} is
   * interrupted, e.g. the button is released, the motors will stop
   *
   * @author Pubert
   */
  /**
   * {@link Command} to move the Pivot Motor forward.
   *
   * @author Pubert
   */
  public Command setabs() {
    return this.run(() -> setit()).withName("Setting");
  }

  public Command runForwardClimbCommand() {
    LaserCan.Measurement measurement = lc.getMeasurement();
    return measurement != null && measurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT
        ? run(() -> setTargetPosition(90.0)).withName("Moving Climb Up")
        : run(() -> m_climbMotor.set(0)).withName("Stoping Climb");
  }

  /**
   * {@link Command} to move the Pivot Motor backward.
   *
   * @author Pubert
   */
  public Command runBackwardClimbCommand() {
    return this.run(() -> setTargetPosition(0.0)).withName("Moving Pivot Backward");
  }

  @Override
  public void periodic() {
    // Display subsystem values
    SmartDashboard.putNumber("Pivot | Pivot | Applied Output", m_climbMotor.getAppliedOutput());
    SmartDashboard.putNumber("Absolute Pos", ae_climbMotor.getPosition());
    SmartDashboard.putNumber("Relative Pos", re_climbMotor.getPosition());
    SmartDashboard.putNumber("Velocity", re_climbMotor.getVelocity());
  }
}

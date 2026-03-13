package frc.robot.motor_ctl;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
  // The shooter motor
  private SparkFlex m_otor;
  private RelativeEncoder e_ncoder;
  private SparkClosedLoopController m_pidController;
  private double kP, kI, kD, kIz, kMaxOutput, kMinOutput, rpmSet;

  private String s_motorName;

  // DriveSubsystem constructor - creates & initializes DriveSubsystem object
  public Flywheel(int canID, SparkBaseConfig config) {
    rpmSet = 2500;
    s_motorName = "Flywheel #" + canID + "/";
    m_otor = new SparkFlex(canID, MotorType.kBrushless);
    m_pidController = m_otor.getClosedLoopController();
    e_ncoder = m_otor.getEncoder();

    kP = 0.0060;
    kI = 0.000001;
    kD = 0.10;
    kIz = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    SmartDashboard.putNumber(s_motorName + " kP", kP);
    SmartDashboard.putNumber(s_motorName + " kI", kI);
    SmartDashboard.putNumber(s_motorName + " kD", kD);

    config.smartCurrentLimit(35);
    config.closedLoop.p(kP).i(kI).d(kD).minOutput(kMinOutput).maxOutput(kMaxOutput).iZone(kIz);
    config.encoder.positionConversionFactor(0.75).velocityConversionFactor(0.75);

    // Apply the configuration settings to the shooter motor SPARK MAX
    // - kResetSafeParameters is used to get the SPARK MAX to a known state. This
    // is useful in case the SPARK MAX is replaced.
    // - kPersistParameters is used to ensure the configuration is not lost when
    // the SPARK MAX loses power. This is useful for power cycles that may occur
    // mid-operation.

    m_otor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    e_ncoder.setPosition(0);
  }

  private void rpmCtl(double rpm) {
    m_pidController.setSetpoint(rpm, ControlType.kVelocity);
  }

  public Command stopMotor() {
    return run(() -> m_otor.setVoltage(0));
  }

  public Command runRPM(double rpm) {
    return run(() -> rpmCtl(rpm));
  }

  public Command runAtSet() {
    return run(() -> rpmCtl(rpmSet));
  }

  private void addToSet(double inc) {
    rpmSet += inc;
  }

  public Command incrSet(double inc) {
    return new InstantCommand(() -> addToSet(inc));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // m_shooterMotor.getOutputCurrent();
    SmartDashboard.putNumber(s_motorName + "Output", m_otor.getAppliedOutput());
    SmartDashboard.putNumber(s_motorName + "Current", m_otor.getOutputCurrent());
    SmartDashboard.putNumber(s_motorName + "Position", e_ncoder.getPosition());
    SmartDashboard.putNumber(s_motorName + "Velocity", e_ncoder.getVelocity());

    SmartDashboard.putBoolean(
        s_motorName + "At Setpoint?",
        m_pidController.getSetpoint() == 0
            ? true
            : (e_ncoder.getVelocity() / m_pidController.getSetpoint()) > 0.95);
    SmartDashboard.putNumber(s_motorName + "Setpoint", m_pidController.getSetpoint());

    SmartDashboard.putNumber(s_motorName + "Internal Setpoint", rpmSet);
  }
}

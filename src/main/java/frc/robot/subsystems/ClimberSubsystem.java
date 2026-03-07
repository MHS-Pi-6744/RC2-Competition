package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import frc.robot.Constants.ClimbSubsystemConstants;
import frc.robot.Constants.ClimbSubsystemConstants;
import frc.robot.Configs;

public class ClimberSubsystem extends SubsystemBase {
    // Initialize intake Spark. We will use open loop control for this

    private SparkMax m_pivotMotor =
        new SparkMax(ClimbSubsystemConstants.kClimbMotorCanId, MotorType.kBrushless);

    private SparkAbsoluteEncoder ae_pivotMotor;
    private RelativeEncoder re_pivotMotor;

    private SparkClosedLoopController p_pivotMotor;

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

        m_pivotMotor.configure(
            Configs.ClimberSubsystem.climbConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        m_setpoint = ClimbSubsystemConstants.ClimbSetPoints.kStartPosition;

        p_pivotMotor = m_pivotMotor.getClosedLoopController();
        
        re_pivotMotor = m_pivotMotor.getEncoder();
        ae_pivotMotor = m_pivotMotor.getAbsoluteEncoder();

        re_pivotMotor.setPosition(ae_pivotMotor.getPosition());

        //re_pivotMotor.setPosition(0);

        setit();

        System.out.println("---> IntakeSubsystem initialized");
    }

    public boolean atTargetPoint() {
        return Math.abs(re_pivotMotor.getPosition() - m_setpoint) < ClimbSubsystemConstants.ClimbSetPoints.kPositionTolerance;
    }

    public void setTargetPosition(double setpos) {
        m_setpoint = setpos;
        moveToSetPoint();
    }

    public void setit() {
      re_pivotMotor.setPosition(ae_pivotMotor.getPosition());
    }

    public void moveToSetPoint() {
        p_pivotMotor.setSetpoint(m_setpoint, ControlType.kMAXMotionPositionControl);
    }

    /**
     * {@link Command} to run the intake motor power {@link Command}. When the
     * {@link Command} is interrupted, e.g. the button is released,
     * the motors will stop
     * 
     * @author Pubert
     */
    /**
     * {@link Command} to move the Pivot Motor forward.
     * 
     * @author Pubert
     */

    public Command setabs(){
      return this.run(
          () -> setit()
      )
      .withName("Setting");
    }
    public Command runForwardPivot() {
        return this.run(
            () -> setTargetPosition(90.0)
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
                () -> setTargetPosition(0.0)
            )
            .withName("Moving Pivot Backward");
    }

    @Override
    public void periodic() {
        // Display subsystem values
        SmartDashboard.putNumber("Pivot | Pivot | Applied Output", m_pivotMotor.getAppliedOutput());
        SmartDashboard.putNumber("Absolute Pos", ae_pivotMotor.getPosition());
        SmartDashboard.putNumber("Relative Pos", re_pivotMotor.getPosition());
    }
}
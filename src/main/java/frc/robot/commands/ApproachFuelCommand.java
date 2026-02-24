package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ObjectDetectionSubsystem;

public class ApproachFuelCommand extends Command {

    private final DriveSubsystem m_robotDrive;
    private final ObjectDetectionSubsystem objectDetection;

    private final PIDController yawPid = new PIDController(0.02, 0.0, 0.0);
    private final PIDController areaPid = new PIDController(0.8, 0.0, 0.0);

    private final double desiredArea = 10.0;  // tune this stuff
    private final double maxForward = 0.6;
    private final double maxRotate = 0.6;

    public ApproachFuelCommand(DriveSubsystem m_robotDrive, ObjectDetectionSubsystem vision) {
        this.m_robotDrive = m_robotDrive;
        this.objectDetection = vision;

        addRequirements(m_robotDrive);

        yawPid.setTolerance(2.0);
    }

    @Override
    public void execute() {

        if (!objectDetection.hasTarget()) {
            m_robotDrive.drive(0.0, 0.0, 0.0, false);
            yawPid.reset();
            areaPid.reset();
            return;
        }

        double yaw = objectDetection.getTargetYaw();
        double area = objectDetection.getTargetArea();

        double rotCmd = MathUtil.clamp(
                yawPid.calculate(yaw, 0.0),
                -maxRotate,
                maxRotate);

        double fwdCmd = MathUtil.clamp(
                areaPid.calculate(area, desiredArea),
                -maxForward,
                maxForward);

        m_robotDrive.drive(fwdCmd, 0.0, rotCmd, false);
    }

    @Override
    public void end(boolean interrupted) {
        m_robotDrive.drive(0.0, 0.0, 0.0, false);
    }

    @Override
    public boolean isFinished() {
        return false; // when button held
    }
}
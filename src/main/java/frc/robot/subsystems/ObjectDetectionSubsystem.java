package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ObjectDetectionSubsystem extends SubsystemBase {

    private final PhotonCamera camera;
    private PhotonTrackedTarget bestTarget;

    public ObjectDetectionSubsystem(String cameraName) {
        camera = new PhotonCamera(cameraName);
    }

    @Override
     public void periodic() {

    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
        bestTarget = null;
        return;
    }

    PhotonPipelineResult latest = results.get(results.size() - 1);

    if (latest.hasTargets()) {
        bestTarget = latest.getBestTarget();
    } else {
        bestTarget = null;
    }
}

    public boolean hasTarget() {
        return bestTarget != null;
    }

    public double getTargetYaw() {
        return hasTarget() ? bestTarget.getYaw() : 0.0;
    }

    public double getTargetArea() {
        return hasTarget() ? bestTarget.getArea() : 0.0;
    }
}

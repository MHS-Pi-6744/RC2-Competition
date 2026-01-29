package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.*;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    PhotonCamera camera;
    PhotonPoseEstimator photonEstimator;

    public Vision() {
        camera = new PhotonCamera(kCameraName);
        photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(kCameraName + " yaw of 3", getTargetYaw(3));
        SmartDashboard.putNumber(kCameraName + " yaw of 25", getTargetYaw(25));
        SmartDashboard.putNumber(kCameraName + " yaw of 26", getTargetYaw(26));
    }

    public double getTargetYaw(int tagID) {
        double yaw = 0.0;
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == tagID) {
                        yaw = target.getYaw();
                        // targetVisible = true;
                    }
                }
            }
        }
        return yaw;
    }
}

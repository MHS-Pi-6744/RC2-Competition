package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.kCameraName;
import static frc.robot.Constants.VisionConstants.kRobotToCam;
import static frc.robot.Constants.VisionConstants.kTagLayout;

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
        getTagYaw(25);
    }
    
    public double getTagYaw(int ID) {
        double yaw = 0.0;
        boolean targetVisible = false;
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == ID) {
                        yaw = target.getYaw();
                        targetVisible = true;
                    }
                }
            }
        }
        SmartDashboard.putBoolean("Tag #" + ID + " Visible", targetVisible);
        SmartDashboard.putNumber("Tag #"+ ID +" Yaw", yaw);
        return yaw;
    }
}

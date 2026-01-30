package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.kCameraName;
import static frc.robot.Constants.VisionConstants.kRobotToCam;
import static frc.robot.Constants.VisionConstants.kTagLayout;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    PhotonCamera camera;
    PhotonPoseEstimator photonEstimator;
    /**
     * This is technically not exactly a supported way to run things.
     * Normally, each tag can only be gotten once due to the way
     * {@link PhotonCamera#getAllUnreadResults()} works.
     * This severely limits us in terms of what we can call and when.
     * However, if we save all of the targets to a list, we remove 
     * the limitations of {@link PhotonCamera#getAllUnreadResults()}
     * by ony calling it once per refresh, as opposed to every call
     * that gets a tag. I came up with this idea while taking a shower.
     *
     * @author MattheDev53
     */
    PhotonTrackedTarget[] targets;

    public Vision() {
        camera = new PhotonCamera(kCameraName);
        photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);
        targets = new PhotonTrackedTarget[32];
    }
    
    @Override
    public void periodic() {
        refreshTags();
        SmartDashboard.putNumber("Tag #25 Yaw", safeGetTagYaw(25));
        SmartDashboard.putBoolean("Tag #25 Visible", getTagVisible(25));
    }
    
    /**
     * Refreshes the list of tags to the most recent reading.
     * @author MattheDev53
     */
    private void refreshTags() {
        
        // Reset the list to make sure old values are cleared
        targets = new PhotonTrackedTarget[32];
        var results = camera.getAllUnreadResults();

        // are there any results?
        if (!results.isEmpty()) {

            // get the most recent result
            var result = results.get(results.size() - 1);

            // are there any targets in the result?
            if (result.hasTargets()) {

                // loop through all of the targets
                for (var target : result.getTargets()) {

                    // put the target in the place it belongs
                    targets[target.getFiducialId()] = target;
                }
            }
        }
    }

    /**
     * Gets an AprilTag's Info from the camera 
     * 
     * @param ID the ID to get
     * @return the {@link PhotonTrackedTarget} that corresponds to the ID passed in
     * @apiNote <b>THIS FUNCTION IS UNSAFE!!</b>
     * Please write a helper function that has a check making sure the tag is not {@code null}
     * to get the property you need.
     * @author MattheDev53
     */
    public PhotonTrackedTarget getTag(int ID) {
        return targets[ID];
    }

    /**
     * Safely gets the Yaw of a certain tag
     * 
     * @param ID The ID of the Tag you wish to get a Yaw value from
     * @return if ({@link #getTag(ID)} == null) return 0.0;
     * Otherwise, the Yaw of the specified AprilTag
     * @author MattheDev53
     */
    public double safeGetTagYaw(int ID) {
        var tag = getTag(ID);
        double yaw = tag == null ? 0.0 : tag.getYaw();
        return yaw;
    }

    /**
     * Gets whether or not a certain tag visible
     * 
     * @param ID The ID of the tag to ask about visibility
     * @return Whether or not the tag is in the most recent result.
     * @author MattheDev53
     */
    public boolean getTagVisible(int ID) {
        return getTag(ID) == null ? false : true;
    }
}

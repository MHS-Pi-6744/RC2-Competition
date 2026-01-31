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
        return getTagSafety(ID) ? 0.0 : getTag(ID).getYaw();
    }

    /**
     * Get whether or not a tag is safe to use.
     * Meant for internal use
     * 
     * @apiNote It is encouraged to use {@link #getTagVisible(ID)}
     * if you want to know about visibility. Yes, that function is
     * literally just a call to this one, but it helps with deciphering
     * your intent in the code. Thank you :3
     * @param ID The ID of the Tag to check
     * @return Is the tag not {@code null}
     * @author MattheDev53
     */
    private boolean getTagSafety(int ID) {
        return getTag(ID) == null ? false : true;
    }

    /**
     * Gets whether or not a certain tag visible.
     * 
     * @param ID The ID of the tag to ask about visibility
     * @return Whether or not the tag is visible
     * @author MattheDev53
     */
    public boolean getTagVisible(int ID) {
        return getTagSafety(ID);
    }

    /**
     * Gets the average Yaw of the <b>visible</b> tags passed in.
     * What this means is if a tag is not visible,
     * it will not be counted in the average.
     * This makes it so that if one wishes to track two tags,
     * but can only see one, the zero value of the non-visible tag
     * does not affect the value of the average.
     * <p>
     * ex #1: If Tag A has a yaw of 30, and Tag B is not visible,
     * the "average" will be 30, because this function ignores
     * the non-visible tags when calculating the divisor.
     * <p> 
     * ex #2: If Tag A has a yaw of 30, and Tag B has a yaw of 0,
     * the average will be 15, because both tags are visible,
     * and will therefore both affect the divisor
     * 
     * @param IDs Array of tags to average
     * @return The average Yaw of all visible tags passed in.
     * If there are no visible tags in the list, this function returns {@code 0.0}
     * @author MattheDev53
     */
    public double getAverageTagsYaw(int[] IDs) {
        
        // Set up variables
        double yawTotal = 0.0;
        int divisor = 0;

        // Loop through all IDs passed in
        for (int ID : IDs) {
            yawTotal += safeGetTagYaw(ID);

            // Tags that are not visible do not affect the divisor
            if (getTagVisible(ID)) divisor++;
        }

        // Avoid Divide by Zero Error
        return divisor == 0 ? 0.0 : yawTotal / divisor;
    }
}

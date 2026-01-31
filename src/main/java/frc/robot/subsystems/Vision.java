package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.kCameraName;
import static frc.robot.Constants.VisionConstants.kRobotToCam;
import static frc.robot.Constants.VisionConstants.kTagLayout;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
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

    Sendable s_tag25 = aprilTagSendable(25);
    Sendable s_tag26 = aprilTagSendable(26);

    public Vision() {
        camera = new PhotonCamera(kCameraName);
        photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);
        targets = new PhotonTrackedTarget[32];
    }
    
    @Override
    public void periodic() {
        refreshTags();
        SmartDashboard.putData("Tag #25", s_tag25);
        SmartDashboard.putData("Tag #26", s_tag26);
        SmartDashboard.putNumber("Average of Tags", getAverageTagsYaw(new int[]{25, 26}));
        SmartDashboard.putNumber("Priority: 25, 26", getTagYawsWithPriority(new int[]{25, 26}));
    }

    /**
     * Creates a sendable for an april tag
     * 
     * @param ID The Tag to create the builder for
     * @author MattheDev53
     */
    private final Sendable aprilTagSendable(int ID) {
        return new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addBooleanProperty("Tag #"+ID+" Visible", () -> getTagVisible(ID), null);

                builder.addDoubleProperty("Tag #"+ID+" Yaw", () -> getTag(ID).getYaw(), null);
                builder.addDoubleProperty("Tag #"+ID+" Skew", () -> getTag(ID).getSkew(), null);
                builder.addDoubleProperty("Tag #"+ID+" Area", () -> getTag(ID).getArea(), null);
                builder.addDoubleProperty("Tag #"+ID+" Pitch", () -> getTag(ID).getPitch(), null);
                builder.addDoubleProperty("Tag #"+ID+" Ambiguity", () -> getTag(ID).getPoseAmbiguity(), null);
            }
        };
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
     * @return the {@link PhotonTrackedTarget}
     * that corresponds to the ID passed in. May return {@code null}.
     * @apiNote <b>THIS FUNCTION IS UNSAFE!!</b>
     * please only use this when you are absolutely certain you
     * want to get an unsafe tag
     * @author MattheDev53
     */
    private PhotonTrackedTarget unsafeGetTag(int ID) {
        return targets[ID];
    }

    /**
     * Gets a Tag based on ID
     * 
     * @param ID The ID of the Tag to get
     * @return The tag with the fiducial ID of the {@code ID} param
     * @apiNote If the requested tag is null, this method will return
     * a completely zeroed out Target. Previously, you would have had
     * to make a function that checks if a given tag is safe or not
     * before returning the value you want. This simplifies things by
     * making it return a fully zeroed out Target. I can't really
     * explain why I didn't do this sooner.
     * @author MattheDev53
     */
    public PhotonTrackedTarget getTag(int ID) {
        return getTagSafety(ID)
        ? unsafeGetTag(ID)
        : new PhotonTrackedTarget(
            0,
            0,
            0,
            0,
            -1,
            0,
            0,
            new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)),
            new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)),
            0,
            List.of(new TargetCorner()),
            List.of(new TargetCorner())
        );
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
        return unsafeGetTag(ID) == null ? false : true;
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
            yawTotal += getTag(ID).getYaw();

            // Tags that are not visible do not affect the divisor
            if (getTagVisible(ID)) divisor++;
        }

        // Avoid Divide by Zero Error
        return divisor == 0 ? 0.0 : yawTotal / divisor;
    }

    /**
     * Gets the Yaw of the Highest priority tag in the given array
     * 
     * @param IDs List of IDs ordered from most significant to least significant
     * @return
     */
    public double getTagYawsWithPriority(int[] IDs) {

        // loop over the array and whichever tag is visible first gets is yaw returned
        for (int ID : IDs) if (getTagVisible(ID)) return getTag(ID).getYaw();

        // if there aren't any tags visible
        return 0.0;
    }
}

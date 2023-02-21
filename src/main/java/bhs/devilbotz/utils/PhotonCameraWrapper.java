package bhs.devilbotz.utils;

import bhs.devilbotz.Constants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.io.IOException;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonCameraWrapper {
  private PhotonCamera photonCamera;
  private PhotonPoseEstimator photonPoseEstimator;

  public PhotonCameraWrapper() {
    // Change the name of your camera here to whatever it is in the PhotonVision UI.
    photonCamera = new PhotonCamera(Constants.VisionConstants.cameraName);

    try {
      // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
      AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      // Create pose estimator
      photonPoseEstimator =
          new PhotonPoseEstimator(
              fieldLayout,
              PoseStrategy.MULTI_TAG_PNP,
              photonCamera,
              Constants.VisionConstants.robotToCam);
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    } catch (IOException e) {
      // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if we don't know
      // where the tags are.
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      photonPoseEstimator = null;
    }
  }

  /**
   * @param prevEstimatedRobotPose The current best guess at robot pose
   * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to create
   *     the estimate
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    if (photonPoseEstimator == null) {
      // The field layout failed to load, so we cannot estimate poses.
      return Optional.empty();
    }
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }

  public Pose2d getClosestTarget(Pose2d robotPose) {
    List tags = photonPoseEstimator.getFieldTags().getTags();
    Pose2d closestTarget = null;
    System.out.println("Number of tags: " + tags.size());

    for (int i = 1; i < tags.size() + 1; i++) {
      Pose2d target = photonPoseEstimator.getFieldTags().getTagPose(i).get().toPose2d();
      if (closestTarget == null) {
        closestTarget = target;
      } else if (target.getTranslation().getDistance(robotPose.getTranslation())
          < closestTarget.getTranslation().getDistance(robotPose.getTranslation())) {
        closestTarget = target;
      }
    }
    return closestTarget;
  }
}

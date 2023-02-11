package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.CAMERA_NAME;
import static frc.robot.Constants.VisionConstants.ROBOT_TO_CAM;

import java.io.IOException;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;

public class Vision {
  private Supplier<Pose2d> poseEstimation;
  private BiConsumer<Pose2d, Double> poseUpdater;
  private PhotonCamera camera;
  private PhotonPoseEstimator photonPoseEstimator;
  private AprilTagFieldLayout aprilTagFieldLayout;

  public Vision(Supplier<Pose2d> poseEstimation, BiConsumer<Pose2d, Double> poseUpdater) {
    this.poseEstimation = poseEstimation;
    this.poseUpdater = poseUpdater;
    camera = new PhotonCamera(CAMERA_NAME);

    aprilTagFieldLayout = loadFieldLayout();
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, 
      camera, ROBOT_TO_CAM);
  }

  private AprilTagFieldLayout loadFieldLayout() {
    try {
      return AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    }
    catch (IOException ioe) {
      ioe.printStackTrace();
    }
    return new AprilTagFieldLayout(null, 0, 0);
  }

  public void update() {
    photonPoseEstimator.setReferencePose(poseEstimation.get());
    photonPoseEstimator.update()
      .ifPresent(estimatedRobotPose -> 
      poseUpdater.accept(estimatedRobotPose.estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds));
  }
  
}

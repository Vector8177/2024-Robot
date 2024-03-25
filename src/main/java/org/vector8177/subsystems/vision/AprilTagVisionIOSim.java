package org.vector8177.subsystems.vision;

import static java.lang.System.arraycopy;
import static org.vector8177.Constants.*;

import org.vector8177.Constants.VisionConstants;
import org.vector8177.Constants.VisionConstants.CameraResolution;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

public class AprilTagVisionIOSim implements AprilTagVisionIO {
  private static final String PhotonCamera = null;

  private final VisionSystemSim visionSim;

  private final PhotonCameraSim flCam;
  private final PhotonPoseEstimator flPoseEstimator;

  private final PhotonCameraSim frCam;
  private final PhotonPoseEstimator frPoseEstimator;

  private Pose3d[] poseArray = new Pose3d[3];
  private double[] timestampArray = new double[3];
  private double[] visionStdArray = new double[9];

  public AprilTagVisionIOSim() {
    PhotonCamera frontL = new PhotonCamera(VisionConstants.frontLeftCameraName);
    PhotonCamera frontR = new PhotonCamera(VisionConstants.fronRightCameraName);

    flPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.frontLeftCameraPosition);
    frPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.frontRightCameraPosition);

    visionSim = new VisionSystemSim("main");
    visionSim.addAprilTags(aprilTagFieldLayout);

    flCam = new PhotonCameraSim(frontL, VisionConstants.OV2311_PROP);
    frCam = new PhotonCameraSim(frontR, VisionConstants.OV2311_PROP);

    visionSim.addCamera(flCam, VisionConstants.frontLeftCameraPosition);
    visionSim.addCamera(frCam, VisionConstants.frontRightCameraPosition);

    flCam.enableDrawWireframe(true);
    frCam.enableDrawWireframe(true);
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    getEstimatedPoseUpdates();
    inputs.visionPoses = poseArray;
    inputs.timestamps = timestampArray;
    inputs.visionStdDevs = visionStdArray;
  }

  @Override
  public void updatePose(Pose2d pose) {
    visionSim.update(pose);
  }

  public void getEstimatedPoseUpdates() {
    Optional<EstimatedRobotPose> pose = flPoseEstimator.update();
    pose.ifPresentOrElse(
        estimatedRobotPose -> {
          poseArray[0] = estimatedRobotPose.estimatedPose;
          timestampArray[0] = estimatedRobotPose.timestampSeconds;
          Matrix<N3, N1> stdDevs =
              getEstimationStdDevs(estimatedRobotPose, CameraResolution.HIGH_RES);
          arraycopy(stdDevs.getData(), 0, visionStdArray, 0, 3);
        },
        () -> {
          poseArray[0] = new Pose3d();
          timestampArray[0] = 0.0;
        });
    pose = frPoseEstimator.update();
    pose.ifPresentOrElse(
        estimatedRobotPose -> {
          poseArray[1] = estimatedRobotPose.estimatedPose;
          timestampArray[1] = estimatedRobotPose.timestampSeconds;
          Matrix<N3, N1> stdDevs =
              getEstimationStdDevs(estimatedRobotPose, CameraResolution.HIGH_RES);
          arraycopy(stdDevs.getData(), 0, visionStdArray, 3, 3);
        },
        () -> {
          poseArray[1] = new Pose3d();
          timestampArray[1] = 0.0;
        });
  }
}

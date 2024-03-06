package frc.robot.subsystems.vision;

import static java.lang.System.arraycopy;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class AprilTagVisionIOReal implements AprilTagVisionIO {
  private final PhotonCamera flCam, frCam;
  private final PhotonPoseEstimator flPoseEstimator, frPoseEstimator;

  private Pose3d[] poseArray = new Pose3d[3];
  private double[] timeStampArray = new double[3];
  private double[] visionStdArray = new double[3];

  public AprilTagVisionIOReal() {
    flCam = new PhotonCamera(VisionConstants.frontLeftCameraName);
    frCam = new PhotonCamera(VisionConstants.fronRightCameraName);

    flPoseEstimator =
        new PhotonPoseEstimator(
            Constants.aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            flCam,
            VisionConstants.frontLeftCameraPosition);
    frPoseEstimator =
        new PhotonPoseEstimator(
            Constants.aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            frCam,
            VisionConstants.frontRightCameraPosition);
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {}

  public void getEstimatedPoseUpdates() {
    Matrix<N3, N1> infiniteStdDevs =
        VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    Optional<EstimatedRobotPose> pose = flPoseEstimator.update();
    pose.ifPresentOrElse(
        estimatedRobotPose -> {
          poseArray[0] = estimatedRobotPose.estimatedPose;
          timeStampArray[0] = estimatedRobotPose.timestampSeconds;
          Matrix<N3, N1> stdDevs = getEstimatedStdDevs(estimatedRobotPose);
          arraycopy(infiniteStdDevs.getData(), 0, visionStdArray, 0, 3);
        },
        () -> {
          poseArray[0] = new Pose3d();
          timeStampArray[0] = 0.0;
          arraycopy(infiniteStdDevs.getData(), 0, visionStdArray, 0, 3);
        });
    pose = flPoseEstimator.update();
    pose.ifPresentOrElse(
        estimatedRobotPose -> {
          poseArray[1] = estimatedRobotPose.estimatedPose;
          timeStampArray[1] = estimatedRobotPose.timestampSeconds;
          Matrix<N3, N1> stdDevs = getEstimatedStdDevs(estimatedRobotPose);
        },
        () -> {
          poseArray[1] = new Pose3d();
          timeStampArray[1] = 0.0;
          arraycopy(infiniteStdDevs.getData(), 0, visionStdArray, 3, 3);
        });
  }
}

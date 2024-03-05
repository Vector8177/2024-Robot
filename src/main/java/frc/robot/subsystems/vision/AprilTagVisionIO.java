package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;

public interface AprilTagVisionIO {
  @AutoLog
  public static class AprilTagVisionIOInputs {
    public Pose3d[] visionPoses =
        List.of(new Pose3d(), new Pose3d(), new Pose3d()).toArray(new Pose3d[0]);
    public double[] timestamps = new double[3];
    public double[] visionStdDevs = new double[9];
  }

  public default void updateInputs(AprilTagVisionIOInputs inputs) {}

  public default void updatePose(Pose2d pose) {}

  default Matrix<N3, N1> getEstimatedStdDevs(EstimatedRobotPose estimatedPose) {
    var estStdDevs = VisionConstants.normalSingleTagStdDev;

    var targets = estimatedPose.targetsUsed;
    int numTags = 0;
    double avgDist = 0;

    for (var target : targets) {
      var tagPose = Constants.aprilTagFieldLayout.getTagPose(target.getFiducialId());

      if (tagPose.isEmpty()) continue;

      numTags++;
      avgDist +=
          tagPose
              .get()
              .toPose2d()
              .minus(estimatedPose.estimatedPose.toPose2d())
              .getTranslation()
              .getNorm();
    }

    if (numTags == 0) return estStdDevs;

    avgDist /= numTags;

    if (numTags > 1 && avgDist > 4) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
    }

    return estStdDevs;
  }
}

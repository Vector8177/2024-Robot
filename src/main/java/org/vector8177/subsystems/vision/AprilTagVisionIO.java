package org.vector8177.subsystems.vision;

import static org.vector8177.Constants.VisionConstants.*;

import org.vector8177.Constants;
import org.vector8177.Constants.VisionConstants.CameraResolution;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

public interface AprilTagVisionIO {
  @AutoLog
  public static class AprilTagVisionIOInputs {
    public Pose3d[] visionPoses = new Pose3d[] {new Pose3d(), new Pose3d()};
    public double[] timestamps = new double[2];
    public double[] visionStdDevs = new double[6];
    // public Pose3d[] visionPoses = new Pose3d[] {new Pose3d()};
    // public double[] timestamps = new double[1];
    // public double[] visionStdDevs = new double[3];
  }

  public default void updateInputs(AprilTagVisionIOInputs inputs) {}

  public default void updatePose(Pose2d pose) {}

  default Matrix<N3, N1> getEstimationStdDevs(
      EstimatedRobotPose estimatedPose, CameraResolution resolution) {
    var estStdDevs =
        switch (resolution) {
          case HIGH_RES -> highResSingleTagStdDev;
          case NORMAL -> normalSingleTagStdDev;
        };

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

    if (numTags > 1
        && avgDist
            > switch (resolution) {
              case HIGH_RES -> 4;
              case NORMAL -> 4;
            }) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      Logger.recordOutput("Vision/StdDev/MultiTagDist", false);
    } else {
      estStdDevs =
          switch (resolution) {
            case HIGH_RES -> highResMultiTagStdDev;
            case NORMAL -> normalMultiTagStdDev;
          };
      Logger.recordOutput("Vision/StdDev/MultiTagDist", true);
    }

    if (numTags == 1
        && avgDist
            > switch (resolution) {
              case HIGH_RES -> 2;
              case NORMAL -> 1;
            }) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      Logger.recordOutput("Vision/StdDev/SingleTagDist", false);

    } else {
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 2));
      Logger.recordOutput("Vision/StdDev/SingleTagDist", true);
    }

    return estStdDevs;
  }
}

package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.VisionConstants.FieldConstants;
import frc.robot.Constants.VisionConstants.PoseEstimation;

public class Camera {
    private final CameraIO cameraIO;

    private final CameraIOInputsAutoLogged cameraInputs = new CameraIOInputAutoLogged();

    private final Transform3d cameraPosition;
    private final AprilTagFieldLayout aprilTagLayout;
    private final PhotonPoseEstimator poseEstimator;

    public Camera(CameraIO camera, Transform3d cameraPosition, AprilTagFieldLayout aprilTagLayout) {
        this.cameraIO = camera;
        this.cameraPosition = cameraPosition;
        this.aprilTagLayout = aprilTagLayout;

        poseEstimator = generatePoseEstimator();
    }

    private PhotonPoseEstimator generatePoseEstimator() {
        PhotonPoseEstimator poseEstimation = new PhotonPoseEstimator(aprilTagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraPosition);

        poseEstimation.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        return poseEstimation;
    }

    public String getCameraName() {
        return cameraInputs.cameraName;
    }

    public void periodic() {
        cameraIO.updateInputs(cameraInputs);
        Logger.processInputs("Cameras/" + getCameraName(), cameraInputs);
    }

    public PhotonPoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    public Optional<EstimatedRobotPose> bob(Pose2d prevEstimatedRobotPose) {
        poseEstimator.setReferencePose(prevEstimatedRobotPose);

        Optional<EstimatedRobotPose> tPose = poseEstimator.update();
        if (!tPose.isEmpty())
        {
            Logger.recordOutput("Odometry/" + getCameraName() + "/RobotPose", tPose.get().estimatedPose);
        }

        return tPose;
        }
    }
}

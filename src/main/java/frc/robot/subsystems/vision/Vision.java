package frc.robot.subsystems.vision;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    private ArrayList<Camera> cameras;
    private AprilTagFieldLayout aprilTagFieldLayout;

    public Vision(CameraIO frontCamLeft) {
        this.cameras = new ArrayList<>();

        try {
            aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

            cameras.add(new Camera(frontCamLeft, VisionConstants.frontLeftCameraPosition, aprilTagFieldLayout));
        } catch (UncheckedIOException e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
        }
    }

    public List<Optional<EstimatedRobotPose>> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        ArrayList<Optional<EstimatedRobotPose>> robotPoses = new ArrayList<>();

        for (Camera camera : cameras) {
            robotPoses.add(camera.getEstimatedPose(prevEstimatedRobotPose));
        }

        return robotPoses;
    }

    @Override
    public void periodic() {
        for (Camera camera : cameras) {
            camera.periodic();
        }
    }

    public void updatePoseAlliance() {
        for (Camera camera : cameras) {
            PhotonPoseEstimator positionEstimation = camera.getPoseEstimator();
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                positionEstimation.getFieldTags().setOrigin(OriginPosition.kBlueAllianceWallRightSide);
            } else {
                positionEstimation.getFieldTags().setOrigin(OriginPosition.kRedAllianceWallRightSide);
            }
        }
    }
}

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public interface CameraIO {
  @AutoLog
  public static class CameraIOInputs {
    public String cameraName = "";
    public boolean connected = false;
    public boolean driverMode = false;
    public PhotonPipelineResult result = new PhotonPipelineResult();
    public double targetTimestamp = 0.0;
    public double[] cameraMatrixData = {};
    public double[] distCoeffsData = {};
  }

  public default void updateInputs(CameraIOInputs inputs) {}

  public default PhotonCamera getCamera() {
    return new PhotonCamera("");
  }
}

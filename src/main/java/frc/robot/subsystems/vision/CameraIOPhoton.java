package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class CameraIOPhoton implements CameraIO {
    private PhotonCamera camera;

    public CameraIOPhoton(String cameraName) { 
        camera = new PhotonCamera(cameraName);

        camera.setDriverMode(false);
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) {
        inputs.cameraName = camera.getName();
        inputs.connected = camera.isConnected();
        inputs.driverMode = camera.getDriverMode();
        PhotonPipelineResult latestResult = camera.getLatestResult();
        inputs.targetData = latestResult.getBestTarget();
        inputs.allTargetData = latestResult.getTargets();
        
    }
}

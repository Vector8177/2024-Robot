// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  private PhotonCamera photonCamera;
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    //name may not be vectorcam (can update later)
    photonCamera = new PhotonCamera("vectorcam");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
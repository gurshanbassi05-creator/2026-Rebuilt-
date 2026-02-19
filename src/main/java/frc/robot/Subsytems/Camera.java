// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
private final UsbCamera Cam;
  /** Creates a new Camera. */
  public Camera() {
Cam = CameraServer.startAutomaticCapture("CAMera", 0);
Cam.setResolution(260, 220);
Cam.setFPS(15);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

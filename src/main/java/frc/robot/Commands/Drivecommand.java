// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsytems.Driveterrain;

public class Drivecommand extends Command {
  //Adding the drive subsystem and controller
  Driveterrain Drivesub;
  XboxController Controller;
  CameraServer Cam;
  public Drivecommand(Driveterrain Drivesub, XboxController Controller) {
    this.Drivesub = Drivesub;
    this.Controller = Controller;
  addRequirements(Drivesub);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Setting variables related to the controllers joysticks
    double RightX = Controller.getRightX();
    double LeftY = Controller.getLeftY();
    //Setting up a controller deadzone by saying if the value of the joysticks is bellow a certain threshold set the varible to 0
    if (Math.abs(LeftY)< 0.05) {
      LeftY = 0;
    }
    if (Math.abs(RightX)< 0.05) {
      RightX = 0;
    }
   
  //Calling the drive method and relating it to the variables
    Drivesub.Drive(LeftY, RightX);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Drivesub.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

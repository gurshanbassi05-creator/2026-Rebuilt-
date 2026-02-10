// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.Drivecommand;
import frc.robot.Commands.Flywheelcommand;
import frc.robot.Commands.Intakecommand;
import frc.robot.Commands.Linearcommand;
import frc.robot.Commands.Autos.DriveForward;
import frc.robot.Commands.Autos.FULLHANGAUTO;
import frc.robot.Subsytems.Camera;
import frc.robot.Subsytems.Driveterrain;
import frc.robot.Subsytems.Flywheel;
import frc.robot.Subsytems.Intake;
import frc.robot.Subsytems.Linearaccuator;

public class RobotContainer {
  //GITHUB test #69420 and 676767676767767676767676
  //Addind the subsystems and controller to the container
  final Driveterrain Drivesub = new Driveterrain();
  //Port 0 means laptop
  final XboxController Controller = new XboxController(0);
  final Linearaccuator Linearsub = new Linearaccuator();
  final Intake Intakesub = new Intake();
  final Flywheel Flywheelsub = new Flywheel();
  //Sendable chooser can let autos be chosen from the smart dashboard
  SendableChooser<Command> chooser = new SendableChooser<>();

  final Camera Camsub = new Camera();
  
      
  public RobotContainer() {
    //Setting the subsytems to the commands (linking them)
    Drivesub.setDefaultCommand(new Drivecommand(Drivesub, Controller));
    Linearsub.setDefaultCommand(new Linearcommand(Linearsub, Controller));
    Intakesub.setDefaultCommand(new Intakecommand(Intakesub, Controller));
    Flywheelsub.setDefaultCommand(new Flywheelcommand(Flywheelsub, Controller));
    //Allowing smartdashboard to contain choices mainly for Auto routines
    SmartDashboard.putData(chooser);
    //Giving the chooser options and getting those actions from the autocommands
    //test
    chooser.addOption("Driveforward", new DriveForward(Drivesub, 5));
    //realauto
    chooser.addOption("FUllhang", new FULLHANGAUTO(Drivesub, Linearsub, Flywheelsub));
    
    configureBindings();
   }

  private void configureBindings() {
    
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}

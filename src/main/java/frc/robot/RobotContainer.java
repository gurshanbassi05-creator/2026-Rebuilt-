// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
   private final CommandXboxController Controller = new CommandXboxController(0); 
  final Linearaccuator Linearsub = new Linearaccuator();
  final Intake Intakesub = new Intake();
  final Flywheel Flywheelsub = new Flywheel();
  //Sendable chooser can let autos be chosen from the smart dashboard
  SendableChooser<Command> chooser = new SendableChooser<>();
  boolean Intakesdeploys, Hoodup;
  final Camera Camsub = new Camera();
  
      
  public RobotContainer() {
    //Setting the subsytems to the commands (linking them)
  Drivesub.setDefaultCommand(new Drivecommand(Drivesub, Controller));
  Intakesub.setDefaultCommand(new Intakecommand(Intakesub, Controller));
  Linearsub.setDefaultCommand(new Linearcommand(Linearsub, Controller));
  Flywheelsub.setDefaultCommand(new Flywheelcommand(Flywheelsub, Controller));
  //Allowing smartdashboard to contain choices mainly for Auto routines
  SmartDashboard.putData(chooser);
   SmartDashboard.putBoolean("bottomswitchhits", Intakesub.Bottomhit());
  SmartDashboard.putBoolean("Topswitchhits", Intakesub.Bottomhit());
  //Giving the chooser options and getting those actions from the autocommands
  //test
  chooser.addOption("Driveforward", new DriveForward(Drivesub, 5));
  //realauto
  chooser.addOption("FUllhang", new FULLHANGAUTO(Drivesub, Linearsub, Flywheelsub));

  configureBindings();
 }

private void configureBindings() {
   
//Configure bindings allows subsystems to act without the use of formal commands. This is an example of a toggleable button 
 Controller.a().toggleOnTrue(new StartEndCommand(
 ()->Intakesub.Intakespeed(0.75),
 ()-> Intakesub.Stop(),
 Intakesub));
//Another togglebale button that allows for the reverse direction
 Controller.b().onTrue(new InstantCommand(()->{ 
 SmartDashboard.putBoolean("Intakedeploed?", Intakesdeploys);
 Intakesdeploys = !Intakesdeploys;
 if (Intakesdeploys) {
 Intakesub.Limitedintakespeed(0.25);
 }
 else{
 Intakesub.Limitedintakespeed(-0.25);
 }
 }));
  //Non toggleable button
Controller.rightBumper().whileTrue(new StartEndCommand(
()->Linearsub.IN_OUT(1),
()->Linearsub.stop(),
Linearsub));

Controller.x().toggleOnTrue(new InstantCommand(()->{
  SmartDashboard.putBoolean("Hoodup", Hoodup);
  if (Hoodup == false) {
    Flywheelsub.resethood();
  }else{
    Flywheelsub.setservo(180);}
  }));
}


public Command getAutonomousCommand() {
  //setting the auto to whatever auto the chooser has selected 
 return chooser.getSelected();
}
}

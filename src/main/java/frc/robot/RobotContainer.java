// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Commands.Drivecommand;
import frc.robot.Commands.Flywheelcommand;
import frc.robot.Commands.Autos.Intakedeployauto;
import frc.robot.Commands.Autos.Intakeretract;
import frc.robot.Commands.Autos.Timeddrive;
import frc.robot.Subsytems.Camera;
import frc.robot.Subsytems.Driveterrain;
import frc.robot.Subsytems.Flywheel;
import frc.robot.Subsytems.Hanging;
import frc.robot.Subsytems.Intake;
public class RobotContainer {
  //Addind the subsystems and controller to the container
  final Driveterrain Drivesub = new Driveterrain();
  private final CommandXboxController Controller = new CommandXboxController(0); 
  final Hanging Hangsub = new Hanging();
  final Intake Intakesub = new Intake();
  final Flywheel Flywheelsub = new Flywheel();
  //General timers;
  //Adding Autofactory for Choreo
private final AutoFactory autoFactory = new AutoFactory(
    Drivesub::getPose, 
    Drivesub::Resetpos, 
    Drivesub::FollowTragectory, 
    false, 
    Drivesub
);
  //Sendable chooser can let autos be chosen from the smart dashboard
  SendableChooser<Command> chooser = new SendableChooser<>();
   final Camera Camsub = new Camera();
   final ChoreoAutos choreoAutos = new ChoreoAutos(autoFactory, Drivesub, Flywheelsub, Hangsub, Intakesub);
  public RobotContainer() {
    
    //Setting the subsytems to the commands (linking them)
  Drivesub.setDefaultCommand(new Drivecommand(Drivesub, Controller));
  Flywheelsub.setDefaultCommand(new Flywheelcommand(Flywheelsub, Controller));
  //Allowing smartdashboard to contain choices mainly for Auto routines 
  //Giving the chooser options and getting those actions from the autocommands
  SmartDashboard.putData(chooser);
  //Putting the values of limit switches
  SmartDashboard.putBoolean("bottomswitchhits", Intakesub.Bottomhit());
  SmartDashboard.putBoolean("Topswitchhits", Intakesub.Bottomhit());
  //Linking the autos to chosers
  chooser.addOption("Choreodrivefrwd", choreoAutos.path1Auto());
  chooser.addOption("TimedShoot", choreoAutos.Timedshoot()); 
  chooser.addOption("TimedFRWD", new Timeddrive(Drivesub, 5,0.5));chooser.addOption("Choreoshoothang", choreoAutos.ShootHang());
  //Datalog started for Sysid testing
 // DataLogManager.start(); 
 // DriverStation.startDataLog(DataLogManager.getLog());
  configureBindings();
 }

private void configureBindings() {
//Configure bindings allows subsystems to act without the use of formal commands. This is an example of a toggleable button
// will spin the intake arm down while spinning the wheels
Controller.a().toggleOnTrue(new StartEndCommand(
 ()->{Intakesub.Intakespeed(-1);
Intakesub.Limitedintakespeed(0.15);},
 ()-> Intakesub.Stop(),
 Intakesub));
 //Will spin the intake arm slowly
  Controller.b().onTrue(new Intakedeployauto(Intakesub));
   //Telscopic tubing code to right and left bumper
  Controller.leftBumper().whileTrue(new StartEndCommand(
    ()->Hangsub.Telscopicspeed(1),
   ()->Hangsub.Stoptele(), 
   Hangsub));
   Controller.rightBumper().whileTrue(new StartEndCommand(
    ()-> Hangsub.Telscopicspeed(-1), 
    ()->Hangsub.Stoptele(), 
   Hangsub ));
   //Using D-Pad to controll elavator
   Controller.povUp().whileTrue(new StartEndCommand(
    ()->Hangsub.Acuatorspeed(1),
    ()->Hangsub.Stopacutor(), 
    Hangsub));
    Controller.povDown().whileTrue(new StartEndCommand(
    ()->Hangsub.Acuatorspeed(-1),
    ()->Hangsub.Stopacutor(), 
    Hangsub));
    //Binding x to intake arm going up
 Controller.x().onTrue(new Intakeretract(Intakesub));
//Another togglebale button that allows for the reverse direction

  //Temporary bindings for Sysid testing
//Controller.a()
    //r.whileTrue(Drivesub.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

//Controller.rightBumper().and(Controller.b())
   // .whileTrue(Drivesub.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

//Controller.rightBumper().and(Controller.x())
    //.whileTrue(Drivesub.sysIdDynamic(SysIdRoutine.Direction.kForward));

//Controller.rightBumper().and(Controller.y())
    //.whileTrue(Drivesub.sysIdDynamic(SysIdRoutine.Direction.kReverse));



}



public Command getAutonomousCommand() {
  //setting the auto to whatever auto the chooser has selected 
 return chooser.getSelected();
}}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Commands.Drivecommand;
import frc.robot.Commands.Flywheelcommand;
import frc.robot.Commands.Intakecommand;
import frc.robot.Commands.Linearcommand;
import frc.robot.Commands.Autos.DriveForward;
import frc.robot.Commands.Autos.FULLHANGAUTO;
import frc.robot.Commands.Autos.Turnto;
import frc.robot.Subsytems.Camera;
import frc.robot.Subsytems.Driveterrain;
import frc.robot.Subsytems.Flywheel;
import frc.robot.Subsytems.Intake;
import frc.robot.Subsytems.Linearaccuator;
public class RobotContainer {
  //Addind the subsystems and controller to the container
  final Driveterrain Drivesub = new Driveterrain();
  private final CommandXboxController Controller = new CommandXboxController(0); 
  final Linearaccuator Linearsub = new Linearaccuator();
  final Intake Intakesub = new Intake();
  final Flywheel Flywheelsub = new Flywheel();
  //General timers;
  private final Timer time = new Timer();
  //Adding Autofactory for Choreo
  private final AutoFactory autoFactory = new AutoFactory(Drivesub::getPose,Drivesub::Resetpos, 
  Drivesub::FollowTragectory, false, Drivesub);
 
  //Sendable chooser can let autos be chosen from the smart dashboard
  SendableChooser<Command> chooser = new SendableChooser<>();
  boolean Intakesdeploys;
  final Camera Camsub = new Camera();
  private final ChoreoAutos choreoAutos = new ChoreoAutos(autoFactory, Flywheelsub);
  public RobotContainer() {
    
    //Setting the subsytems to the commands (linking them)
  Drivesub.setDefaultCommand(new Drivecommand(Drivesub, Controller));
  Intakesub.setDefaultCommand(new Intakecommand(Intakesub, Controller));
  Linearsub.setDefaultCommand(new Linearcommand(Linearsub, Controller));
  Flywheelsub.setDefaultCommand(new Flywheelcommand(Flywheelsub, Controller));
  //Allowing smartdashboard to contain choices mainly for Auto routines 
  //Giving the chooser options and getting those actions from the autocommands
  SmartDashboard.putData(chooser);
  //Putting the values of limit switches
  SmartDashboard.putBoolean("bottomswitchhits", Intakesub.Bottomhit());
  SmartDashboard.putBoolean("Topswitchhits", Intakesub.Bottomhit());
  //Linking the autos to chosers
  chooser.addOption("Driveforward", new DriveForward(Drivesub, 5));
  chooser.addOption("FUllhang", new FULLHANGAUTO(Drivesub, Linearsub, Flywheelsub));
  chooser.addOption("turnto", new Turnto(Drivesub, 90 ));
  chooser.addOption("Choreodrivefrwd", choreoAutos.path1Auto());
  chooser.addOption("Shootdeitve", choreoAutos.shootdrive());
  //Datalog started for Sysid testing
  DataLogManager.start(); 
  DriverStation.startDataLog(DataLogManager.getLog());
  configureBindings();
 }

private void configureBindings() {
//Configure bindings allows subsystems to act without the use of formal commands. This is an example of a toggleable button 
Controller.a().toggleOnTrue(new StartEndCommand(
 ()->Intakesub.Intakespeed(-0.75),
 ()-> Intakesub.Stop(),
 Intakesub));
//Another togglebale button that allows for the reverse direction
Controller.b().onTrue(new InstantCommand(()->{
  Intakesub.Limitedintakespeed(0.45);
  if (Intakesub.Bottomhit()) {
  Intakesub.Limitedintakestop();}}));
  //Temporary bindings for Sysid testing
Controller.a()
    .whileTrue(Drivesub.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

Controller.rightBumper().and(Controller.b())
    .whileTrue(Drivesub.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

Controller.rightBumper().and(Controller.x())
    .whileTrue(Drivesub.sysIdDynamic(SysIdRoutine.Direction.kForward));

Controller.rightBumper().and(Controller.y())
    .whileTrue(Drivesub.sysIdDynamic(SysIdRoutine.Direction.kReverse));

//Example of a command that stops when given a certain trigger
Controller.x().onTrue(new InstantCommand(
 ()-> { Intakesub.Limitedintakespeed(-1);
  if (Intakesub.Tophit()) {
  Intakesub.Limitedintakestop();}}));
  //Non toggleable button
Controller.rightBumper().whileTrue(new StartEndCommand(
()->Linearsub.IN_OUT(-0.45),
()->Linearsub.stop(),
Linearsub));

Controller.leftBumper().whileTrue(new StartEndCommand(
()->Linearsub.IN_OUT(0.45),
()->Linearsub.stop(),
Linearsub));

Controller.rightTrigger().whileTrue(new StartEndCommand(
  ()->Flywheelsub.FlywheelSpeed(-0.75),
  ()->Flywheelsub.Stop(),
  Flywheelsub));
//Examples for a delay added in a command
Controller.rightTrigger().onTrue(new InstantCommand(
  ()->time.reset()));

Controller.rightTrigger().whileTrue(new StartEndCommand(
  ()->{
  if (time.get()>3) {
    Flywheelsub.Kickspeed(1);}
  else{
    Flywheelsub.Kickspeed(0);
    }},
  ()->Flywheelsub.Kickspeed(0),
  Flywheelsub));
}



public Command getAutonomousCommand() {
  //setting the auto to whatever auto the chooser has selected 
 return chooser.getSelected();
}}


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsytems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Intakecommand extends Command {
  //Defing the subsytem the command will be using
   Intake Intakesub;
   double motorspeed;
   //Defing the controller
  CommandXboxController controller;
  //makes 2 booleans to toggle buttons 
  boolean Intakeon, Intakedeploy;
  //Creating a double to link it to the encoder
  //double X;
 //Links the subsytem and controller to the command
  public Intakecommand(Intake Intakesub, CommandXboxController controller) {
    //allows it to be used in the execure methods
  this.Intakesub = Intakesub;

  this.controller = controller;
  //Adding requirements binding the subsytem to the command
  addRequirements(Intakesub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Set the booleans to to be false thus turning Intake off and folded in at the begginng of the match
    Intakeon = false;
    Intakedeploy = false;
    //Reseting encoder so it starts at 0 aka folded postion for the intake 
    //Intakesub.resetencoder();
   // Intakesub.Setpoisiton(0);
  }

  // Called every time the scheduler runs while the command is scheduled. At 0.2 seconds per tick
  @Override
  public void execute() {
  SmartDashboard.putBoolean("bottomswitchhits", Intakesub.Bottomhit());
  SmartDashboard.putBoolean("Topswitchhits", Intakesub.Tophit());
    
      controller.b().onTrue(new InstantCommand(
        ()->{
          Intakedeploy = !Intakedeploy;
        }));
    
    if (Intakesub.Bottomhit() == false && Intakedeploy ) {
      Intakesub.Limitedintakestop();
    }
   if (Intakesub.Tophit() == false && Intakedeploy == false) {
    Intakesub.Limitedintakestop();
   }
    //Setting X equal to the current encoder position
    //X = Intakesub.Getencoderposition();
    
    //The idea is to have the intake work on 1 toggable button press. 
    //Press A once to turn it on, Then press A again to turn it off
    // if (controller.getAButtonPressed()) {
      //"!" flips the boolean which turns Intakeon true
    // Intakeon = !Intakeon;
     //If intakeon is true then set the speed to 1
   // if (Intakeon) {
    //  Intakesub.Intakespeed(-0.75); 
    // }
     //else set the speed to 0 when the boolean is false (when the button is pused again)
   // else{
 // Intakesub.Stop();
    //  }
    // }
    // if (controller.getBButtonPressed()) {
    //  Intakedeploy = !Intakedeploy;
    //  if (Intakedeploy) {
    //    Intakesub.Limitedintakespeed(0.25);
    ///  }
    //  if (Intakedeploy == false) {
    //    Intakesub.Limitedintakespeed(-0.25);
     // }
    
     }
     

//Comments are for neo encoder position DO NOT DELETE

     //if (controller.getBButtonPressed()) {
      //Same togglable system as with the intake
     // Intakedeploy = !Intakedeploy;
    // if (Intakedeploy) {
      //Setting the position of the encoder which will rotate the motor to the value set times
      //  Intakesub.Setpoisiton(20);
     // }
     // else{
     //reset the intake postion back to inside 
       // Intakesub.Setpoisiton(0);
      //}
     

     //Smartdashboard can record the rotation for testing 
   //  SmartDashboard.putNumber("Number of rotations", X);
    


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

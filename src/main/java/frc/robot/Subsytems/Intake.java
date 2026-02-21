// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Intake extends SubsystemBase{
  SparkMax Intake, Intakedeploy;
  RelativeEncoder encoder;
  SparkClosedLoopController pidController;
  SparkMaxConfig config;
   DigitalInput Toplimitswitch, BottomLimitswitch;
 
  /** Creates a new Intake. */
  public Intake() { 
    Intake = new SparkMax(8, MotorType.kBrushed);
    //Defining neo
    //Intakedeploy = new SparkMax(6, MotorType.kBrushless);
    Intakedeploy = new SparkMax(6, MotorType.kBrushed);
    //Using the dio ports on the RIO to initalize the digital input
    Toplimitswitch = new DigitalInput(0);
    BottomLimitswitch = new DigitalInput(1);          


    //Creates an encoder and sets it to the neo encoder
    //encoder = Intakedeploy.getEncoder();
    //PID controller allows precise movement and this sets it equal to the motors controller
    //pidController = Intakedeploy.getClosedLoopController();
    //Configs configure motor settings
    //config = new SparkMaxConfig();
    //setting a p value which is like strength and allows the motor to spin
    //config.closedLoop.p(0.095);
    //Xonfiguring the motor for saftey and linking it to the config
   // Intakedeploy.configure(config, com.revrobotics.ResetMode.kResetSafeParameters , 
     //com.revrobotics.PersistMode.kPersistParameters);
  }

  //Limit switch methods
    public boolean Tophit(){
      return Toplimitswitch.get();
    }
   public boolean Bottomhit(){
    return BottomLimitswitch.get();
   }
  
    public void Limitedintakespeed(double speed){
     Intakedeploy.set(speed);
    }
    public void Limitedintakestop(){
    Intakedeploy.stopMotor();
    }
    
    //Encoder methods
  //Creates method to get the current postion of the encoder
 // public double Getencoderposition(){
  //return encoder.getPosition();
  //}
  //Uses the PID controller to let the encoder value influence the position of the motor in number of roations
 // public void Setpoisiton(double Rotaitons){
 // pidController.setSetpoint(Rotaitons, SparkMax.ControlType.kPosition);
 // }
  //Method to set the encoder position to 0
  //public void resetencoder(){
  //encoder.setPosition(0);
  //}
  //Method assiging the intake rollers a variable that can is their speed
  public void Intakespeed(double speed){
  Intake.set(speed);
  }
  //DOing the same for the neo motor
  public void Stop(){
  Intake.stopMotor();
  }
  @Override
  public void periodic() {
}
}

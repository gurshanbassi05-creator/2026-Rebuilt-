// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Driveterrain extends SubsystemBase {
    //Defining Motors
  SparkMax Frontleft, Frontright, Backleft, Backright;
  //Defining the Drivebase
  DifferentialDrive Drivebase;
  public Driveterrain() {
//Initalising Motors (Giving the CAN ID)
    Frontleft = new SparkMax(1, MotorType.kBrushed);
    Frontright = new SparkMax(2, MotorType.kBrushed);
    Backleft = new SparkMax(3, MotorType.kBrushed);
    Backright = new SparkMax(4, MotorType.kBrushed);
    //Tekking the drive base what values to use
    Drivebase = new DifferentialDrive(this::Leftmotors, this::Rightmotors);
  }
  //Grouping Leftmotors
  public void Leftmotors(double speed){
    Frontleft.set(speed);
    Backleft.set(speed);
  }
  //Grouping Rightmotors
  public void Rightmotors(double speed){
    Frontright.set(-speed);
    Backright.set(-speed);
  }
  //Creating a Method to set the Drivebase with 2 doubles
  public void Drive(double speed, double turn){
    Drivebase.arcadeDrive(speed, turn);
  }
  //A method to stop the drivebase
  public void Stop(){
    Drivebase.arcadeDrive(0, 0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

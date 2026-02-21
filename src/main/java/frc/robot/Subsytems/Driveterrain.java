// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Driveterrain extends SubsystemBase {
   
  
    //Defining Motors
  final SparkMax Frontleft, Frontright, Backleft, Backright;
  final double Feetperotation = 1.57;
   double Reset =0;
  //Defining encoders
  AnalogEncoder Leftencoder, Rightencoder;
  //Gyro
  final AHRS gyro;
  //Defining the Drivebase
  DifferentialDrive Drivebase;
  
  public Driveterrain() {
    
//Initalising Motors (Giving the CAN ID)
    Frontleft = new SparkMax(1, MotorType.kBrushed);
    Frontright = new SparkMax(2, MotorType.kBrushed);
    Backleft = new SparkMax(3, MotorType.kBrushed);
    Backright = new SparkMax(4, MotorType.kBrushed);
    //Defineing gyro
    gyro = new AHRS(NavXComType.kMXP_SPI);
    //Defining encoders
    Leftencoder = new AnalogEncoder(1);
    Rightencoder = new AnalogEncoder(2);
    //Tekking the drive base what values to use
    Drivebase = new DifferentialDrive(this::Leftmotors, this::Rightmotors);
  }
  public void resetencoders(){
Reset = Leftencoder.get();
Reset = Rightencoder.get();
  }
  public double LeftPositioninfeet(){
return Leftencoder.get()*Feetperotation;
  }
  public double RightPositioninfeet(){
    return Rightencoder.get()*Feetperotation;
  }
 
  public Rotation2d Heading(){
    return Rotation2d.fromRadians(gyro.getYaw());
  }
    public void resetHeading(){
    gyro.reset();
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
    SmartDashboard.putNumber("Gyroheading", Heading().getRadians());
  
  }
}

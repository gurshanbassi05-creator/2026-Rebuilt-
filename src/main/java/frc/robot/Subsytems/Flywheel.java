// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
  TalonSRX Flywheel;
 SparkMax Kick;
  /** Creates a new Flywheel. */
  public Flywheel() {
  Kick = new SparkMax(6, MotorType.kBrushless);
  Flywheel = new TalonSRX(10);

  }
  public void Speed(double speed){
    Flywheel.set(TalonSRXControlMode.PercentOutput, speed);
    Kick.set(speed);
    
  }
  
  public void Stop(){
    Flywheel.set(TalonSRXControlMode.PercentOutput, 0);
    Kick.stopMotor();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

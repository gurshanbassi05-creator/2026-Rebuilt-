// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
  TalonSRX Flywheel, Kick;
  //Servos
  /** Creates a new Flywheel. */
  public Flywheel() {
  Kick = new TalonSRX(12);
  Flywheel = new TalonSRX(10);
  }
  public void FlywheelSpeed(double speed){
  Flywheel.set(TalonSRXControlMode.PercentOutput, speed);
  }
  public void Kickspeed(double speed){
      Kick.set(TalonSRXControlMode.PercentOutput, speed);  
  }

  public void Stop(){
  Flywheel.set(TalonSRXControlMode.PercentOutput, 0);
  Kick.set(TalonSRXControlMode.PercentOutput, 0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

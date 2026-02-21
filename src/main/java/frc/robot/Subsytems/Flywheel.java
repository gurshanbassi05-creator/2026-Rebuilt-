// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
  TalonSRX Flywheel, Kick;
  //Servos
  Servo Lefthood, Righthood;
  /** Creates a new Flywheel. */
  public Flywheel() {
  Kick = new TalonSRX(12);
  Flywheel = new TalonSRX(10);
  //Pwm channels used
  Lefthood = new Servo(0);
  Righthood = new Servo(1);
  
  }
  //The servos have a 180 degree range the neutral position is midway at 90
  public void resethood(){
  Lefthood.setAngle(90);
  Righthood.setAngle(90);
  }
  //Method to allow the servos to rotate atthe desired angle between 0 and 180
  public void setservo(double angle){
  Lefthood.setAngle(angle);
  Righthood.setAngle(-angle);

  }
  public double servoangle(){
    return Lefthood.getPosition();
  }
  public void Speed(double speed){
  Flywheel.set(TalonSRXControlMode.PercentOutput, speed);
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

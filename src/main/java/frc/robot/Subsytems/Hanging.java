// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hanging extends SubsystemBase {
  TalonSRX Telscopic, Acuator;

  /** Creates a new Hanging. */
  public Hanging() {
    Telscopic = new TalonSRX(9);
    Acuator = new TalonSRX(11);

  }
  public void Telscopicspeed(double Speed){
Telscopic.set(ControlMode.PercentOutput, Speed);
  }
  public void Acuatorspeed(double Speed){
    Acuator.set(ControlMode.PercentOutput, Speed);
  }
  public void Stoptele(){
    Telscopic.set(ControlMode.PercentOutput, 0);
  }
  public void Stopacutor(){
Acuator.set(ControlMode.PercentOutput, 0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Linearaccuator extends SubsystemBase {
  //Defining motors that use the victorSP motor controller
TalonSRX Climb;


  public Linearaccuator() {
   //Channel is the port on the PWM port on the roboRIo
    Climb = new TalonSRX(11);
      
  }
 
  public void IN_OUT(double speed){
Climb.set(TalonSRXControlMode.PercentOutput, speed);
  }
   
  public void stop(){    
Climb.set(TalonSRXControlMode.PercentOutput, 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

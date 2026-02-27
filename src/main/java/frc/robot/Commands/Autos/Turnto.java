// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsytems.Driveterrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Turnto extends Command {
  /** Creates a new TurnTO. */
  Driveterrain drive;
  //change value "Kp" for rough controller and change value "ki" for smooth controll Kd will predict the error and is ectra if time is found 
  PIDController pid = new PIDController(0.350, 2, 0);
  Rotation2d angle;
  double MAX_OUTPUT = 0.55;
  
  public Turnto(Driveterrain drive, double angle) {
    this.drive = drive;
    this.angle = Rotation2d.fromDegrees(angle);

    pid.enableContinuousInput(-Math.PI,Math.PI);
    //PID.setIzone is the "ki" range/ sensetivity aka when robot is within this range start compling values
    //max error before it starts to compile values
    pid.setIZone(Units.degreesToRadians(20));
    //min error before it starts to compile values
    pid.setTolerance(Units.degreesToRadians(5));
    pid.setTolerance(2, 0);
    pid.setSetpoint(angle);
    addRequirements(drive);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetHeading();
    pid.reset();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //added by smort
    double Gyroturn = -pid.calculate(drive.Heading().getRadians(), angle.getRadians());
    Gyroturn = Math.copySign(Math.min(Math.abs(Gyroturn), MAX_OUTPUT), Gyroturn);
    drive.Drive(0, Gyroturn);
    //display values
    SmartDashboard.putNumber("Turn pid output", Gyroturn);
    SmartDashboard.putBoolean("Finishedturn", pid.atSetpoint());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("FINISHED");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsytems.Driveterrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Turnto extends Command {
  Driveterrain Drivesub;
  //Pid controller allows percise movement by compling error values 
  PIDController PID = new PIDController(1.25, 1, 0);
  //angel of turn
  Rotation2d Angleinradians;
  //maxiumim push
  double MAX_OUTPUT = 0.725;
  /** Creates a new Turnto. */
  public Turnto(Driveterrain Drivesub, double Angleinradians) {
    this.Drivesub = Drivesub;
    this.Angleinradians = Rotation2d.fromRadians(Angleinradians);
    //sets range from -180 to 180 degree turns 
    PID.enableContinuousInput(-Math.PI, Math.PI);
    //Izone is a range where the gyro will add up the error and increse the power accordingly when it is less than the Izone. 
    PID.setIZone(8*Math.PI/180);
    //Tolerance is the error that is allowed before stopping
    PID.setTolerance(Math.PI/90);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Drivesub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Reset the gyro
    Drivesub.resetHeading();
    PID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Calculating where the gyro is and where it has to be
     double Gyroturn = -PID.calculate(Drivesub.Heading().getRadians(), Angleinradians.getRadians());
     //Theres alot in this line 
     //Math.copy sign returns a value with both magnitude- a value and the sign- positive or negative in the second item 
     //Math.min selects 2 double values and returns the lower one this ensures that the gyro never exceeds the max out put
    Gyroturn = Math.copySign(Math.min(Math.abs(Gyroturn), MAX_OUTPUT), Gyroturn);
    //Telling the drive base to drive to whatever gyro turn is set as
    Drivesub.Drive(0, Gyroturn);
    //display values
    SmartDashboard.putNumber("Turn pid output", Gyroturn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("FINISHED");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //When the heading is within the tollarable zone stop the command
    return PID.atSetpoint();
  }
}

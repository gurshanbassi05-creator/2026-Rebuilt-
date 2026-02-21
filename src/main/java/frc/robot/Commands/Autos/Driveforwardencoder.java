// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsytems.Driveterrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Driveforwardencoder extends Command {
  Driveterrain Drivesub;
  PIDController PID = new PIDController(1.25, 1, 0);
  double distance;
  
  /** Creates a new Driveforwardencoder. */
  public Driveforwardencoder(Driveterrain Drivesub, double distance) {
this.Drivesub = Drivesub;
this.distance = distance;
 
PID.setTolerance(0.05);
PID.setIZone(0.5);
addRequirements(Drivesub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   Drivesub.resetencoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double Leftpositon = Drivesub.LeftPositioninfeet();
    double movement = PID.calculate(Leftpositon, distance);
    Drivesub.Drive(movement, 0);
    SmartDashboard.putNumber("powertodrive", movement);
    SmartDashboard.putNumber("Encoderpositon in feet", Leftpositon);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return PID.atSetpoint();
  }
}

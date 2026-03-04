// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsytems.Driveterrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Timeddrive extends Command {
  Driveterrain Drivesub;
  private final Timer Time = new Timer();
  private final double seconds;
  private final double speed;
  /** Creates a new Timeddrive. */
  public Timeddrive(Driveterrain Drivesub, double seconds, double speed) {
    this.Drivesub = Drivesub;
    this.speed = speed;
    this.seconds = seconds;
    addRequirements(Drivesub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Time.reset();
    Time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Time.get() < seconds) {
      Drivesub.Drive(speed, 0);
    }
    if (Time.get()>seconds) {
      Drivesub.Stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Time.get() > seconds;
  }
}

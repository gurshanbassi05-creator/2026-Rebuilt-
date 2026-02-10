// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsytems.Driveterrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveForward extends Command {
  Driveterrain Drivesub;
  double Seconds;
  Timer timer;

  public DriveForward(Driveterrain Drivesub, double Seconds) {
this.Drivesub = Drivesub;
this.Seconds = Seconds;
timer = new Timer();
addRequirements(Drivesub);

    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() < Seconds) {
      Drivesub.Drive(0.25, 0);
    }
   if (timer.get() >= Seconds) {
    Drivesub.Stop();
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= Seconds;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsytems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Intakerollers extends Command {
  Intake Intakesub;
  double seconds;
  Timer times;
  /** Creates a new Intakerollers. */
  public Intakerollers(Intake Intakesub, double seconds) {
    this.Intakesub = Intakesub;
    this.seconds = seconds;
    addRequirements(Intakesub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    times.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (times.get()<seconds) {
      Intakesub.Intakespeed(-0.75);
    }
    else{
      Intakesub.Stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return times.get()>seconds;
  }










































}

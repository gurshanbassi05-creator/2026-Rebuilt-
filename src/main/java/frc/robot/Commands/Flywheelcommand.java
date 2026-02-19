// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsytems.Flywheel;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Flywheelcommand extends Command {
  Flywheel flywheelsub;
  CommandXboxController controller;
  double RightTrigger;
  /** Creates a new Flywheelcommand. */
  public Flywheelcommand(Flywheel flywheelsub, CommandXboxController controller) {
    this.flywheelsub = flywheelsub;
    this.controller = controller;
    addRequirements(flywheelsub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flywheelsub.resethood();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RightTrigger = controller.getRightTriggerAxis();
    flywheelsub.Speed(40*RightTrigger );
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsytems.Linearaccuator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Lineardown extends Command {
  Linearaccuator Linearsub;
  double seconds;
  Timer timer;
  public Lineardown( Linearaccuator Linearsub) {
    this.Linearsub = Linearsub;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Linearsub);
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
    if(timer.get() < 4){
Linearsub.IN_OUT(-1);
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 4;
  }
}

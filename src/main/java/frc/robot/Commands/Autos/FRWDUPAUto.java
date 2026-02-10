// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsytems.Driveterrain;
import frc.robot.Subsytems.Linearaccuator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FRWDUPAUto extends ParallelCommandGroup {
  Driveterrain Drivesub;
  Linearaccuator Linearsub;
  /** Creates a new FRWDUPAUto. */
  public FRWDUPAUto(Driveterrain Drivesub, Linearaccuator Linearsub) {
    this.Drivesub = Drivesub;
    this.Linearsub = Linearsub;
    addRequirements(Drivesub, Linearsub);
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
     new DriveForward(Drivesub, 4), 
     new LinearslideUP(Linearsub, 4)
    );
  }
}

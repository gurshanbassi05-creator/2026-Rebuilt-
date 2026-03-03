// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import static frc.robot.generated.ChoreoTraj.*;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.Autos.Flywheelshoot;
import frc.robot.Subsytems.Driveterrain;
import frc.robot.Subsytems.Flywheel;

/** Add your docs here. */
public class ChoreoAutos {
   private final Flywheel Flywheelsub;
   private final Driveterrain Drivesub;
private final AutoFactory autofactory;
 public ChoreoAutos (AutoFactory autofactory,Driveterrain Drivesub, Flywheel Flywheelsub){
 this.Flywheelsub = Flywheelsub;
this.autofactory = autofactory;
this.Drivesub = Drivesub;
    }

    public Command path1Auto() {
        
        var routine = autofactory.newRoutine("Path1Auto");
        var traj = Drvie360.asAutoTraj(routine);
        var fullcommand = Commands.sequence(
        traj.resetOdometry(),
        traj.cmd() );
        fullcommand.addRequirements(Drivesub);
        return fullcommand; 
}
    
    public Command shootdrive(){
        var Path3 = autofactory.newRoutine("Shootplan");
        var trag1 = Drvie360.asAutoTraj(Path3);
        
       return Commands.sequence(
        trag1.resetOdometry(),
        new Flywheelshoot(Flywheelsub, 2),
        trag1.cmd()
        );
     
    }
}

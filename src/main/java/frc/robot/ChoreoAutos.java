// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import static frc.robot.generated.ChoreoTraj.*;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.Autos.Flywheelshoot;
import frc.robot.Subsytems.Flywheel;

/** Add your docs here. */
public class ChoreoAutos {
   private final Flywheel Flywheelsub;
    private final AutoFactory autofactory;
    public ChoreoAutos (AutoFactory autofactory, Flywheel Flywheelsub){
 this.Flywheelsub = Flywheelsub;
this.autofactory = autofactory;
    }

    public Command path1Auto() {
        var routine = autofactory.newRoutine("Path1Auto");
        var traj = Path_1.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                traj.resetOdometry(),
                traj.cmd()
            )
        );
        return routine.cmd();
    }
    public Command shootdrive(){
        var Path3 = autofactory.newRoutine("Shootplan");
        var trag1 = Path_1.asAutoTraj(Path3);
        
       return Commands.sequence(
        new Flywheelshoot(Flywheelsub, 5),
        trag1.resetOdometry(),
        trag1.cmd()
        );
     
    }
}

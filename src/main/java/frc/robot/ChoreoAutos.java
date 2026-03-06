// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import static frc.robot.generated.ChoreoTraj.*;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.Autos.Flywheelshoot;
import frc.robot.Commands.Autos.Intakedeployauto;
import frc.robot.Commands.Autos.Intakerollers;
import frc.robot.Commands.Autos.L1Hang;
import frc.robot.Commands.Autos.Timeddrive;
import frc.robot.Subsytems.Driveterrain;
import frc.robot.Subsytems.Flywheel;
import frc.robot.Subsytems.Hanging;
import frc.robot.Subsytems.Intake;

/** Add your docs here. */
public class ChoreoAutos {
   private final Flywheel Flywheelsub;
   private final Hanging Hangsub;
   private final Driveterrain Drivesub;
   private final Intake Intakesub;
private final AutoFactory autofactory;
 public ChoreoAutos (AutoFactory autofactory,Driveterrain Drivesub, Flywheel Flywheelsub, Hanging Hangsub, Intake Intakesub){
 this.Flywheelsub = Flywheelsub;
this.autofactory = autofactory;
this.Intakesub = Intakesub;
this.Drivesub = Drivesub;
this.Hangsub = Hangsub;
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
public Command ShootHang(){
    var routine = autofactory.newRoutine("HubtoHang");
    var traj = HubtoHang.asAutoTraj(routine);
return Commands.sequence(
    traj.resetOdometry(),
    new Flywheelshoot(Flywheelsub, 3),
    traj.cmd(),
    new L1Hang(Hangsub, 5)
);}
public Command Leftshoothang(){
    var routine = autofactory.newRoutine("Lefthub");
    var Traj = Lefttohub.asAutoTraj(routine);
    var Hang = HubtoHang.asAutoTraj(routine);
    var fullcommand = Commands.sequence(
        Traj.resetOdometry(),
    Traj.cmd(),
    new Flywheelshoot(Flywheelsub, 3),
    Hang.resetOdometry(),
    Hang.cmd(),
    new L1Hang(Hangsub, 10)
    );
    fullcommand.addRequirements(Drivesub);
    return fullcommand;
}
  public Command ShootBallsShoot(){
    var routine = autofactory.newRoutine("ShootBallsShoot");
    var hubtoballs = HubtoBalls.asAutoTraj(routine);
    var BallstoHUb = BallstoHUB.asAutoTraj(routine);
    return Commands.sequence(hubtoballs.resetOdometry(),
    new Flywheelshoot(Flywheelsub, 3),
    hubtoballs.cmd(),
    new Intakedeployauto(Intakesub),
    new Intakerollers(Intakesub, 03),
    BallstoHUb.resetOdometry(),
    BallstoHUb.cmd(),
    new Flywheelshoot(Flywheelsub, 3));
  }
    public Command Timedshoot(){
        return Commands.sequence(
new Timeddrive(Drivesub, 0.75),
new Flywheelshoot(Flywheelsub, 5)
        );
    }
   public Command L1hang(){
    return  Commands.sequence( new Timeddrive(Drivesub, 1.5),
    new L1Hang(Hangsub, 10));
   }
    }


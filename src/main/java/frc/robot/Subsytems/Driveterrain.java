// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import choreo.trajectory.DifferentialSample;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ThriftyAbsoluteEncoder;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.*;
// Remove: import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog; (This is old)
public class Driveterrain extends SubsystemBase {
  private final LTVUnicycleController controller = new LTVUnicycleController(0.2);  

  private final AnalogEncoder ecoderleft; 
    private final AnalogEncoder encoderright; 
  private ThriftyAbsoluteEncoder Leftencoder = new ThriftyAbsoluteEncoder(2);
  private ThriftyAbsoluteEncoder Rightencoder = new ThriftyAbsoluteEncoder(3);
  private final double kMaxSpeed = 3.0;
    //Defining Motors
final SparkMax Frontleft, Frontright, Backleft, Backright;
  //double to convert the encoders position from rotations to feet public final double Drive_To_gearratio = 1;
public final double Wheel_Radius_inmeter = Units.inchesToMeters(3);
public final double wheel_Diameter = Units.inchesToMeters(6);
public final double Trackwidth_inmeter = Units.inchesToMeters(22.5);
private final double Circumference = Math.PI*wheel_Diameter;
private final double Drive_To_gearratio = 1;
public final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
private final SysIdRoutine Routine = new SysIdRoutine(new SysIdRoutine.Config(),
 new SysIdRoutine.Mechanism(this::Drievvoltage, this::Logmotors, this));
private final Field2d m_field = new Field2d();
public final DifferentialDriveKinematics KINE = new DifferentialDriveKinematics(Trackwidth_inmeter);
public final DifferentialDriveOdometry Estimator = new DifferentialDriveOdometry(
  Heading(),
  Leftpose(),
  Rightpose());
  //Defining encoders
  //Defining the Drivebase
  DifferentialDrive Drivebase;
  
  public Driveterrain() {
//Initalising Motors (Giving the CAN ID)
  Frontleft = new SparkMax(1, MotorType.kBrushed);
  Frontright = new SparkMax(2, MotorType.kBrushed);
  Backleft = new SparkMax(3, MotorType.kBrushed);
  Backright = new SparkMax(4, MotorType.kBrushed);
  Drivebase = new DifferentialDrive(this::Leftmotors, this::Rightmotors);
  encoderright = new AnalogEncoder(1);
  ecoderleft = new AnalogEncoder(0);
//ress
}
public double analogleft(){
  return ecoderleft.get();
}
public double analogright(){
return encoderright.get();
}
 public void Drievvoltage(Voltage V){
  Frontleft.setVoltage(-V.in(Volts));
  Frontright.setVoltage(V.in(Volts));
 }
 public void Logmotors(SysIdRoutineLog Log){
Log.motor("Leftmotors")
.voltage(Volts.of(Frontleft.getBusVoltage()*Frontleft.getAppliedOutput()))
.linearPosition(Meters.of(Leftpose()))
.linearVelocity(MetersPerSecond.of(Leftencoder.getvelocity()*Circumference));
Log.motor("Rightmotor").voltage(Volts.of(Frontright.getBusVoltage()*Frontright.getAppliedOutput()))
.linearPosition(Meters.of(Rightpose()))
.linearVelocity(MetersPerSecond.of(Rightencoder.getvelocity()*Circumference));
}


  public Pose2d getPose(){
return Estimator.getPoseMeters();

  }
 public void FollowTragectory(DifferentialSample sample) {
    Pose2d pose = Estimator.getPoseMeters();
    
    // 1. Get the planned speeds from the Choreo file
    ChassisSpeeds targetChassisSpeeds = sample.getChassisSpeeds();
    DifferentialDriveWheelSpeeds targetWheelSpeeds = KINE.toWheelSpeeds(targetChassisSpeeds);

    // 2. Calculate the CORRECTION (Feedback)
    // We use the vx and omega from targetChassisSpeeds
    ChassisSpeeds correctionSpeeds = controller.calculate(
        pose, 
        sample.getPose(), 
        targetChassisSpeeds.vxMetersPerSecond, 
        targetChassisSpeeds.omegaRadiansPerSecond
    );
    DifferentialDriveWheelSpeeds correctionWheelSpeeds = KINE.toWheelSpeeds(correctionSpeeds);

    // 3. Add Feedforward + Feedback
    double leftFinal = targetWheelSpeeds.leftMetersPerSecond + correctionWheelSpeeds.leftMetersPerSecond;
    double rightFinal = targetWheelSpeeds.rightMetersPerSecond + correctionWheelSpeeds.rightMetersPerSecond;

    // 4. Send to motors
    Setwheelspeeds(leftFinal, rightFinal);
}
  public void Setwheelspeeds(double LMS, double RMS){
Leftmotors(LMS/kMaxSpeed);
Rightmotors(RMS/kMaxSpeed);
  }
  public Rotation2d getGyroheading(){
    return new Rotation2d(gyro.getAngle()*Math.PI/180);
  }
public void Resetpos(Pose2d Pose){
  Estimator.resetPose(Pose);
}
  
  public void Zeroposition(){
    Leftencoder.setRelativePosition(0);
    Rightencoder.setRelativePosition(0);
   Estimator.resetPose(new Pose2d(0,0, new Rotation2d()));
  }

  public double Leftpose(){
  return Leftencoder.getRelativeRotations()*Drive_To_gearratio*wheel_Diameter;
}
  public double Rightpose(){
  return Rightencoder.getRelativeRotations()*Drive_To_gearratio*wheel_Diameter;
}
 //A method that gets the hedding/angle that the rbot is at
  public Rotation2d Heading(){
    return Rotation2d.fromDegrees(gyro.getYaw());
  }
  //Method that resets the gyros values
    public void resetHeading(){
    gyro.reset();
  }
  //Grouping Leftmotors
  public void Leftmotors(double speed){
    Frontleft.set(speed);
    Backleft.set(speed);
  }
  //Grouping Rightmotors
  public void Rightmotors(double speed){
    Frontright.set(-speed);
    Backright.set(-speed);
  }
  //Creating a Method to set the Drivebase with 2 doubles
  public void Drive(double speed, double turn){
    Drivebase.arcadeDrive(speed, turn);
  }
  //A method to stop the drivebase
  public void Stop(){
    Drivebase.arcadeDrive(0, 0);
  }
public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return Routine.quasistatic(direction) .beforeStarting(() -> 
    Drivebase.setSafetyEnabled(false)) // Turn off safety
        .andThen(() -> Drivebase.setSafetyEnabled(true));      // Turn it back on after
}

/** Returns a command to run the dynamic (step voltage) test */
public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return Routine.dynamic(direction)
     .beforeStarting(() -> Drivebase.setSafetyEnabled(false)) // Turn off safety
        .andThen(() -> Drivebase.setSafetyEnabled(true));      // Turn it back on after
}

  @Override
  public void periodic() {
  Estimator.update(getGyroheading(), Leftpose(), Rightpose());
  SmartDashboard.putNumber("Analogleft", analogleft());
  SmartDashboard.putNumber("analogright", analogright());
  SmartDashboard.putNumber("leftencoder", Leftpose());
  SmartDashboard.putNumber("Rightencoder", Rightpose());
  SmartDashboard.putNumber("Odom (X)", Estimator.getPoseMeters().getX());
  SmartDashboard.putNumber("Odom (Y)", Estimator.getPoseMeters().getY());
  SmartDashboard.putData("Field", m_field);
  m_field.setRobotPose(Estimator.getPoseMeters());
  SmartDashboard.putNumber("Gyroheading", Heading().getDegrees());
   
  }
}

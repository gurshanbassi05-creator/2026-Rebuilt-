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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ThriftyAbsoluteEncoder;

public class Driveterrain extends SubsystemBase {
   private final LTVUnicycleController controller = new LTVUnicycleController(0.02);
  private ThriftyAbsoluteEncoder Leftencoder = new ThriftyAbsoluteEncoder(9);
  private ThriftyAbsoluteEncoder Rightencoder = new ThriftyAbsoluteEncoder(8);
    //Defining Motors
final SparkMax Frontleft, Frontright, Backleft, Backright;
  //double to convert the encoders position from rotations to feet public final double Drive_To_gearratio = 1;
public final double Wheel_Radius_inmeter = Units.inchesToMeters(3);
public final double Trackwidth_inmeter = Units.inchesToMeters(22.5);
private final double Drive_To_gearratio = 1;
public final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);  
//private final SimpleMotorFeedforward Motorfeed = new SimpleMotorFeedforward(Wheel_Radius_inmeter, Trackwidth_inmeter, Drive_To_gearratio);
public final DifferentialDriveKinematics KINE = new DifferentialDriveKinematics(Trackwidth_inmeter);
public final DifferentialDriveOdometry Estimator = new DifferentialDriveOdometry(
  getGyroheading(),
  Leftpose(),
  Rightpose());
  //Defining encoders
  
  //Gyro
  //Defining the Drivebase
  DifferentialDrive Drivebase;
  
  public Driveterrain() {
//Initalising Motors (Giving the CAN ID)
    Frontleft = new SparkMax(1, MotorType.kBrushed);
    Frontright = new SparkMax(2, MotorType.kBrushed);
    Backleft = new SparkMax(3, MotorType.kBrushed);
    Backright = new SparkMax(4, MotorType.kBrushed);
    //Defineing gyro
    
    //Defining encoders and the analogIN port on the rio they are connected to
   
    //Tekking the drive base what values to use
  Drivebase = new DifferentialDrive(this::Leftmotors, this::Rightmotors);
  }
  
  public Pose2d getPose(){
return Estimator.getPoseMeters();
  }
  public void FollowTragectory(DifferentialSample Sample){
Pose2d pose = Estimator.getPoseMeters();
ChassisSpeeds ff = Sample.getChassisSpeeds();
ChassisSpeeds Speeds = controller.calculate(pose, Sample.getPose(), ff.vxMetersPerSecond, ff.omegaRadiansPerSecond);
DifferentialDriveWheelSpeeds wheelSpeeds = KINE.toWheelSpeeds(Speeds);
Drive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }
  public Rotation2d getGyroheading(){
    return new Rotation2d(gyro.getAngle());
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
  return Leftencoder.getRelativeRotations()*Drive_To_gearratio*Math.PI*Wheel_Radius_inmeter;
}
  public double Rightpose(){
  return Rightencoder.getRelativeRotations()*Drive_To_gearratio*Math.PI*Wheel_Radius_inmeter;
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
  @Override
  public void periodic() {
  Estimator.update(getGyroheading(), Leftpose(), Rightpose());
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Odom (X)", Estimator.getPoseMeters().getX());
    SmartDashboard.putNumber("Odom (Y)", Estimator.getPoseMeters().getY());
    SmartDashboard.putNumber("Gyroheading", Heading().getDegrees());
   
  }
}

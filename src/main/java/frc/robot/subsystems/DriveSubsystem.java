// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */

  private CANSparkMax leftMotorFront = new CANSparkMax(DriveConstants.leftFrontMotorId, MotorType.kBrushless); 
  private CANSparkMax leftMotorBack = new CANSparkMax(DriveConstants.leftBackMotorId, MotorType.kBrushless); 
  private CANSparkMax rightMotorFront = new CANSparkMax(DriveConstants.rightFrontMotorId, MotorType.kBrushless); 
  private CANSparkMax rightMotorBack = new CANSparkMax(DriveConstants.rightBackMotorId, MotorType.kBrushless); 

  private RelativeEncoder rightFrontEncoder = rightMotorFront.getEncoder(); 
  private RelativeEncoder leftFrontEncoder = leftMotorFront.getEncoder(); 

  DifferentialDrive differentialDrive = new DifferentialDrive(leftMotorFront, rightMotorFront); 

  private AHRS navx; 
  

  public DriveSubsystem() {

    leftMotorFront.restoreFactoryDefaults(); 
    leftMotorBack.restoreFactoryDefaults(); 
    rightMotorFront.restoreFactoryDefaults(); 
    rightMotorBack.restoreFactoryDefaults(); 

    rightMotorFront.setInverted(true);
    leftMotorFront.setInverted(false);
    
    leftMotorBack.follow(leftMotorFront); 
    rightMotorBack.follow(rightMotorFront); 

    rightFrontEncoder.setPosition(0); 
    leftFrontEncoder.setPosition(0); 

    navx = new AHRS(SPI.Port.kMXP); 
    navx.reset();
    navx.zeroYaw();

  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("left encoder", leftEncoderPosition()); 
    SmartDashboard.putNumber("right encoder", rightEncoderPosition()); 
    SmartDashboard.putNumber("left velocity", leftEncoderVelocity()); 
    SmartDashboard.putNumber("right velocity", rightEncoderVelocity()); 

    SmartDashboard.putNumber("left wheel inches", leftEncoderToInches()); 
    SmartDashboard.putNumber("right wheel inches", rightEncoderToInches()); 
  
    SmartDashboard.putNumber("yaw", getYaw()); 
    SmartDashboard.putNumber("pitch", getPitch()); 
    SmartDashboard.putNumber("roll", getRoll()); 
  }

  public void setBrakeMode(){
    leftMotorBack.setIdleMode(IdleMode.kBrake); 
    leftMotorFront.setIdleMode(IdleMode.kBrake); 
    rightMotorFront.setIdleMode(IdleMode.kBrake); 
    rightMotorBack.setIdleMode(IdleMode.kBrake); 
  }

  public void setCoastMode(){
    leftMotorBack.setIdleMode(IdleMode.kCoast); 
    leftMotorFront.setIdleMode(IdleMode.kCoast); 
    rightMotorBack.setIdleMode(IdleMode.kCoast); 
    rightMotorFront.setIdleMode(IdleMode.kCoast); 
  }

  public void resetEncoders(){
    rightFrontEncoder.setPosition(0); 
    leftFrontEncoder.setPosition(0); 
  }

  public double leftEncoderPosition(){
    return leftFrontEncoder.getPosition(); 
  }

  public double rightEncoderPosition(){
    return rightFrontEncoder.getPosition(); 
  }

  public double leftEncoderVelocity(){
    return leftFrontEncoder.getVelocity(); 
  }

  public double rightEncoderVelocity(){
    return rightFrontEncoder.getVelocity();  
  }

  public double rightEncoderToInches(){
    return rightEncoderPosition() * DriveConstants.revToInch; 
  }

  public double leftEncoderToInches(){
    return leftEncoderPosition() * DriveConstants.revToInch; 
  }

  public double averageEncoderDistanceInInches(){
    return (leftEncoderToInches() + rightEncoderToInches())/2; 
  }

  public double getRoll(){
    return navx.getRoll(); 
  }

  // navx orientation causes pitch to be roll 
  public double getPitch(){
    return navx.getRoll(); 
  }

  public double getYaw(){
    return navx.getPitch(); 
  }

  public void zeroHeading(){
    navx.reset(); 
    navx.zeroYaw(); 
  }

  public void set(double drive, double turn){
    differentialDrive.arcadeDrive(drive, turn);
  }

  public void setTank(double left, double right){
    differentialDrive.tankDrive(left, right);
  }

  public void stop(){
    differentialDrive.stopMotor();
  }
}

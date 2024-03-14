// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  private final String DRIVE_PREFIX = "SmartDashboard/Drive"; 

  /** Creates a new DriveSubsystem. */

  private CANSparkMax leftMotorFront = new CANSparkMax(DriveConstants.leftFrontMotorId, MotorType.kBrushless); 
  private CANSparkMax leftMotorBack = new CANSparkMax(DriveConstants.leftBackMotorId, MotorType.kBrushless); 
  private CANSparkMax rightMotorFront = new CANSparkMax(DriveConstants.rightFrontMotorId, MotorType.kBrushless); 
  private CANSparkMax rightMotorBack = new CANSparkMax(DriveConstants.rightBackMotorId, MotorType.kBrushless); 

  private RelativeEncoder rightFrontEncoder = rightMotorFront.getEncoder(); 
  private RelativeEncoder leftFrontEncoder = leftMotorFront.getEncoder(); 

  private RelativeEncoder rightBackEncoder = rightMotorBack.getEncoder(); 
  private RelativeEncoder leftBackEncoder = leftMotorBack.getEncoder(); 

  private DifferentialDrive differentialDrive = new DifferentialDrive(leftMotorFront, rightMotorFront); 

  private AHRS navx; 
  

  public DriveSubsystem() {

    leftMotorFront.restoreFactoryDefaults(); 
    leftMotorBack.restoreFactoryDefaults(); 
    rightMotorFront.restoreFactoryDefaults(); 
    rightMotorBack.restoreFactoryDefaults(); 

    rightMotorFront.setInverted(false);
    leftMotorFront.setInverted(true);
    
    leftMotorBack.follow(leftMotorFront); 
    rightMotorBack.follow(rightMotorFront); 

    rightFrontEncoder.setPosition(0); 
    leftFrontEncoder.setPosition(0); 

    leftBackEncoder.setPosition(0); 
    rightBackEncoder.setPosition(0); 

    navx = new AHRS(SPI.Port.kMXP); 
    navx.reset();
    navx.zeroYaw();

  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(DRIVE_PREFIX + "left encoder", leftEncoderPosition()); 
    SmartDashboard.putNumber("right encoder", rightEncoderPosition());
    
    
    SmartDashboard.putNumber(DRIVE_PREFIX + "left velocity", leftEncoderVelocity()); 
    SmartDashboard.putNumber(DRIVE_PREFIX + "right velocity", rightEncoderVelocity()); 
    SmartDashboard.putNumber(DRIVE_PREFIX + "right back velocity", rightBackEncoderVelocity()); 
    SmartDashboard.putNumber(DRIVE_PREFIX + "left back velocity", leftBackEncoderVelocity()); 

    SmartDashboard.putNumber(DRIVE_PREFIX + "left wheel inches", leftEncoderToInches()); 
    SmartDashboard.putNumber(DRIVE_PREFIX + "right wheel inches", rightEncoderToInches()); 
  
    SmartDashboard.putNumber(DRIVE_PREFIX + "yaw", getYaw()); 
    SmartDashboard.putNumber(DRIVE_PREFIX + "pitch", getPitch()); 
    SmartDashboard.putNumber(DRIVE_PREFIX + "roll", getRoll()); 

        
    SmartDashboard.putNumber(DRIVE_PREFIX + "RELATIVE", getAngle0to360()); 
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

  public double leftBackEncoderVelocity(){
    return leftBackEncoder.getVelocity(); 
  }

  public double rightEncoderVelocity(){
    return rightFrontEncoder.getVelocity();  
  }

  public double rightBackEncoderVelocity(){
    return rightBackEncoder.getVelocity(); 
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
    return navx.getPitch(); 
  }

  public double getYaw(){
    return navx.getYaw(); 
  }

  public double getAngle0to360(){
   
    double angle = 0; 

    if(getYaw() < 180 && getYaw() > 0){
      angle = getYaw(); 
    }

    else if(getYaw() < 0){
      angle  = 180 + (getYaw() + 180); 
    }

    else if(getYaw() > 395.5){
      angle = 0; 
    }


    return angle; 
  }


  public double getHeadingError(double desiredHeading){

    double error; 

    if(desiredHeading > getAngle0to360() + 180){
      error = (desiredHeading - getAngle0to360()) - 360; 
    }

    else if(desiredHeading < getAngle0to360() - 180){
      error = (desiredHeading - getAngle0to360()) + 360;  
    }

    else{
      error = desiredHeading - getAngle0to360(); 
    }

    return error; 
  }

  public void zeroHeading(){
    navx.reset(); 
    navx.zeroYaw(); 
  }


  public void setLeftPIDF(double p, double i, double d, double f){
    leftMotorFront.getPIDController().setP(p); 
    leftMotorFront.getPIDController().setI(i); 
    leftMotorFront.getPIDController().setD(d); 
    leftMotorFront.getPIDController().setFF(f); 
  }

  public void setRightPIDF(double p, double i, double d, double f){
    rightMotorFront.getPIDController().setP(p); 
    rightMotorFront.getPIDController().setI(i); 
    rightMotorFront.getPIDController().setD(d); 
    rightMotorFront.getPIDController().setFF(f);
  }

  public void leftOutPutConstraints(double min, double max){
    leftMotorFront.getPIDController().setOutputRange(min, max); 
  }

  public void rightOutPutConstraints(double min, double max){
    rightMotorFront.getPIDController().setOutputRange(min, max); 
  }

  public void setRightVelocityMode(){
    rightMotorFront.getPIDController().setReference(0, ControlType.kVelocity); 
  }

  public void setLeftVelocityMode(){
    leftMotorFront.getPIDController().setReference(0, ControlType.kVelocity); 
  }

  public void setRightPowerMode(){
    rightMotorFront.getPIDController().setReference(0, ControlType.kVoltage); 
  }

  public void setLeftPowerMode(){
    leftMotorFront.getPIDController().setReference(0, ControlType.kVoltage);
  }

  public void setVelocity(double leftVelocity, double rightVelocity){
    leftMotorFront.getPIDController().setReference(leftVelocity, ControlType.kVelocity); 
    // leftMotorBack.getPIDController().setReference(leftVelocity, ControlType.kVelocity); 
    rightMotorFront.getPIDController().setReference(rightVelocity, ControlType.kVelocity); 
    // rightMotorBack.getPIDController().setReference(rightVelocity, ControlType.kVelocity); 
    

    // differentialDrive.tankDrive(leftVelocity, rightVelocity);
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

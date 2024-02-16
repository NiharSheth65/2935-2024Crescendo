// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private CANSparkMax intakeLeftMotor = new CANSparkMax(IntakeConstants.intakeLeftMotorId, MotorType.kBrushless); 
  private CANSparkMax intakeRightMotor = new CANSparkMax(IntakeConstants.intakeRightMotorId, MotorType.kBrushless); 

  private RelativeEncoder intakeEncoderLeft = intakeLeftMotor.getEncoder(); 
  private RelativeEncoder intakeEncoderRight = intakeRightMotor.getEncoder(); 
  
  public IntakeSubsystem() {
    
    intakeLeftMotor.restoreFactoryDefaults(); 
    intakeRightMotor.restoreFactoryDefaults(); 

    intakeLeftMotor.setInverted(false);
    intakeRightMotor.setInverted(true);

    intakeEncoderLeft.setPosition(0);
    intakeEncoderRight.setPosition(0);  

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
    SmartDashboard.putNumber("intake velocity", intakeLeftVelocity()); 
  }


  public void setBrakeMode(){
    intakeLeftMotor.setIdleMode(IdleMode.kBrake); 
    intakeRightMotor.setIdleMode(IdleMode.kBrake); 
  }

  public void setCoastMode(){
    intakeLeftMotor.setIdleMode(IdleMode.kCoast); 
    intakeRightMotor.setIdleMode(IdleMode.kCoast); 
  }

  public void resetEncoders(){
    intakeEncoderLeft.setPosition(0);  
    intakeEncoderRight.setPosition(0); 
  }

  public double intakeLeftPosition(){
    return intakeEncoderLeft.getPosition(); 
  }

  public double intakeLeftVelocity(){
    return intakeEncoderLeft.getVelocity(); 
  }

  public double intakeRightPosition(){
    return intakeEncoderRight.getPosition(); 
  }

  public double intakeRightVelocity(){
    return intakeEncoderRight.getVelocity(); 
  }

  // public double intakeRightCurrent(){
  //   return intakeEncoderRight.getC
  // }

  public void setLeftIntakePIDF(double p, double i, double d, double f){
    intakeLeftMotor.getPIDController().setP(p); 
    intakeLeftMotor.getPIDController().setI(i); 
    intakeLeftMotor.getPIDController().setD(d); 
    intakeLeftMotor.getPIDController().setFF(f); 
  }

  public void intakeOutPutConstraints(double min, double max){
    intakeLeftMotor.getPIDController().setOutputRange(min, max); 
  }

  public void setIntakeVelocityMode(){
    intakeLeftMotor.getPIDController().setReference(0, ControlType.kVelocity); 
    intakeRightMotor.getPIDController().setReference(0, ControlType.kVelocity); 
  }

  public void setIntakePowerMode(){
    intakeLeftMotor.getPIDController().setReference(0, ControlType.kVoltage); 
    intakeRightMotor.getPIDController().setReference(0, ControlType.kVoltage); 
  }

  public void setVelocity(double intakeVelocity){
    intakeLeftMotor.getPIDController().setReference(intakeVelocity, ControlType.kVelocity); 
    intakeRightMotor.getPIDController().setReference(intakeVelocity, ControlType.kVelocity); 
  }

  public void setIntake(double leftIntakeSpeed, double rightIntakeSpeed){
    intakeLeftMotor.set(leftIntakeSpeed);
    intakeRightMotor.set(rightIntakeSpeed);
  }

  public void stop(){
    intakeLeftMotor.stopMotor();
    intakeRightMotor.stopMotor(); 
  }
}

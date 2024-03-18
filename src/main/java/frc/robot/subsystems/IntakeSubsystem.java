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

  private CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorId, MotorType.kBrushless); 
  private RelativeEncoder intakeEncoder = intakeMotor.getEncoder(); 

  public IntakeSubsystem() {
    
    intakeMotor.restoreFactoryDefaults(); 

    intakeMotor.setInverted(true);

    intakeEncoder.setPosition(0);

    intakeMotor.burnFlash();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
    SmartDashboard.putNumber("intake velocity", intakeMotorVelocity()); 
    SmartDashboard.putNumber("intake current", intakeMotorCurrent()); 
    SmartDashboard.putBoolean("intaked", hasIntaked()); 


  }


  public void setBrakeMode(){
    intakeMotor.setIdleMode(IdleMode.kBrake); 
  }

  public void setCoastMode(){
    intakeMotor.setIdleMode(IdleMode.kCoast); 
  }

  public void resetEncoders(){
    intakeEncoder.setPosition(0);  
  }

  public double intakeMotorPosition(){
    return intakeEncoder.getPosition(); 
  }

  public double intakeMotorVelocity(){
    return intakeEncoder.getVelocity(); 
  }

  public double intakeMotorCurrent(){
    return intakeMotor.getOutputCurrent(); 
  }
  // public double intakeRightCurrent(){
  //   return intakeEncoderRight.getC
  // }


  public boolean hasIntaked(){
    
    if(intakeMotorCurrent() > 20){
      return true; 
    }

    else{
      return false; 
    }
  }

  public void setLeftIntakePIDF(double p, double i, double d, double f){
    intakeMotor.getPIDController().setP(p); 
    intakeMotor.getPIDController().setI(i); 
    intakeMotor.getPIDController().setD(d); 
    intakeMotor.getPIDController().setFF(f); 
  }

  public void intakeOutPutConstraints(double min, double max){
    intakeMotor.getPIDController().setOutputRange(min, max); 
  }

  public void setIntakeVelocityMode(){
    intakeMotor.getPIDController().setReference(0, ControlType.kVelocity); 
  }

  public void setIntakePowerMode(){
    intakeMotor.getPIDController().setReference(0, ControlType.kVoltage); 
  }

  public void setVelocity(double intakeVelocity){
    intakeMotor.getPIDController().setReference(intakeVelocity, ControlType.kVelocity); 
  }

  public void setIntake(double leftIntakeSpeed, double rightIntakeSpeed){
    intakeMotor.set(leftIntakeSpeed);
  }

  public void stop(){
    intakeMotor.stopMotor();
  }
}

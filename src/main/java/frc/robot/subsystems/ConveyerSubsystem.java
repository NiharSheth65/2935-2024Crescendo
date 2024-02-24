// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.conveyerConstants;

public class ConveyerSubsystem extends SubsystemBase {


  private CANSparkMax conveyerMotor = new CANSparkMax(conveyerConstants.conveyerMotorId, MotorType.kBrushless); 

  private RelativeEncoder conveyerEncoder = conveyerMotor.getEncoder(); 
  
  /** Creates a new ConveyerSubsystem. */
  public ConveyerSubsystem() {
    conveyerMotor.restoreFactoryDefaults(); 

    conveyerMotor.setInverted(false);

    conveyerEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setBrakeMode(){
    conveyerMotor.setIdleMode(IdleMode.kBrake); 
  }

  public void setCoastMode(){
    conveyerMotor.setIdleMode(IdleMode.kCoast); 
  }

  public void resetEncoders(){
    conveyerEncoder.setPosition(0);  
  }

  public double conveyerMotorPosition(){
    return conveyerEncoder.getPosition(); 
  }

  public double conveyerMotorVelocity(){
    return conveyerEncoder.getVelocity(); 
  }

  public double conveyerMotorCurrent(){
    return conveyerMotor.getOutputCurrent(); 
  }
  // public double conveyerRightCurrent(){
  //   return conveyerEncoderRight.getC
  // }

  public void setLeftconveyerPIDF(double p, double i, double d, double f){
    conveyerMotor.getPIDController().setP(p); 
    conveyerMotor.getPIDController().setI(i); 
    conveyerMotor.getPIDController().setD(d); 
    conveyerMotor.getPIDController().setFF(f); 
  }

  public void conveyerOutPutConstraints(double min, double max){
    conveyerMotor.getPIDController().setOutputRange(min, max); 
  }

  public void setconveyerVelocityMode(){
    conveyerMotor.getPIDController().setReference(0, ControlType.kVelocity); 
  }

  public void setconveyerPowerMode(){
    conveyerMotor.getPIDController().setReference(0, ControlType.kVoltage); 
  }

  public void setVelocity(double conveyerVelocity){
    conveyerMotor.getPIDController().setReference(conveyerVelocity, ControlType.kVelocity); 
  }

  public void setConveyer(double conveyerSpeed){
    conveyerMotor.set(conveyerSpeed);
  }

  public void stop(){
    conveyerMotor.stopMotor();
  }
}

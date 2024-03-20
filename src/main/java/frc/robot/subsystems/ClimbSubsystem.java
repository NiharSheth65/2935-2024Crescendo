// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.IntakeConstants;

public class ClimbSubsystem extends SubsystemBase {

  private CANSparkMax climbMotor = new CANSparkMax(ClimbConstants.climbMotorID, MotorType.kBrushless); 
  private RelativeEncoder climbEncoder = climbMotor.getEncoder(); 


  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    climbMotor.restoreFactoryDefaults(); 
    climbMotor.setInverted(false);
    climbEncoder.setPosition(0);
    setBrakeMode();

    climbMotor.burnFlash();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setBrakeMode(){
    climbMotor.setIdleMode(IdleMode.kBrake); 
  }

  public void setCoastMode(){
    climbMotor.setIdleMode(IdleMode.kCoast); 
  }

  public void resetEncoders(){
    climbEncoder.setPosition(0);  
  }

  public double climbMotorPosition(){
    return climbEncoder.getPosition(); 
  }


  public double climbMotorCurrent(){
    return climbMotor.getOutputCurrent(); 
  }

  public double climbMotorVelocity(){
    return climbEncoder.getVelocity(); 
  }

  
  public void setTopclimbPIDF(double p, double i, double d, double f){
    climbMotor.getPIDController().setP(p); 
    climbMotor.getPIDController().setI(i); 
    climbMotor.getPIDController().setD(d); 
    climbMotor.getPIDController().setFF(f); 
  }


  public void climbOutPutConstraints(double min, double max){
    climbMotor.getPIDController().setOutputRange(min, max); 
  }

  public void setclimbVelocityMode(){
    climbMotor.getPIDController().setReference(0, ControlType.kVelocity); 
  }

  public void setclimbPowerMode(){
    climbMotor.getPIDController().setReference(0, ControlType.kVoltage); 
  }

  public void setVelocity(double climbVelocity){
    climbMotor.getPIDController().setReference(climbVelocity, ControlType.kVelocity); 
  }

  public void setClimb(double climbSpeed){
    climbMotor.set(climbSpeed);
  }

  public void stop(){
    climbMotor.stopMotor();
  }
}

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.conveyerConstants;

public class ConveyerSubsystem extends SubsystemBase {


  private CANSparkMax conveyerTopMotor = new CANSparkMax(conveyerConstants.conveyerTopMotorId, MotorType.kBrushless); 
  private CANSparkMax conveyerBottomMotor = new CANSparkMax(conveyerConstants.conveyerBottomMotorId, MotorType.kBrushless); 

  private RelativeEncoder conveyerTopEncoder = conveyerTopMotor.getEncoder(); 
  private RelativeEncoder conveyerBottomEncoder = conveyerBottomMotor.getEncoder(); 

  // zero - lowest 
  // one -> middle 
  // two - highest 
  private DigitalInput intakeSwitch1 = new DigitalInput(IntakeConstants.switchOnePort); 
  private DigitalInput intakeSwitch2 = new DigitalInput(IntakeConstants.switchTwoPort); 
  private DigitalInput intakeSwitch3 = new DigitalInput(IntakeConstants.switchThreePort); 
  
  /** Creates a new ConveyerSubsystem. */
  public ConveyerSubsystem() {

    conveyerTopMotor.restoreFactoryDefaults(); 
    conveyerBottomMotor.restoreFactoryDefaults(); 

    conveyerTopMotor.setInverted(false);
    conveyerBottomMotor.setInverted(true);

    conveyerTopEncoder.setPosition(0);
    conveyerBottomEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("conveyer top current", conveyerTopMotorCurrent());  
    SmartDashboard.putNumber("conveyer bototm current", conveyerBottomMotorCurrent());  

    SmartDashboard.putBoolean("switch one", intakeSwitchOneValue()); 
    SmartDashboard.putBoolean("switch two", intakeSwitchTwoValue()); 
    SmartDashboard.putBoolean("switch three", intakeSwitchThreeValue()); 
  }

  public boolean intakeSwitchOneValue(){
    return intakeSwitch1.get(); 
  }

  public boolean intakeSwitchTwoValue(){
    return intakeSwitch2.get(); 
  }

  public boolean intakeSwitchThreeValue(){
    return intakeSwitch3.get(); 
  }

  public void setBrakeMode(){
    conveyerTopMotor.setIdleMode(IdleMode.kBrake); 
    conveyerBottomMotor.setIdleMode(IdleMode.kBrake); 
  }

  public void setCoastMode(){
    conveyerTopMotor.setIdleMode(IdleMode.kCoast); 
    conveyerBottomMotor.setIdleMode(IdleMode.kCoast); 
  }

  public void resetEncoders(){
    conveyerTopEncoder.setPosition(0);  
    conveyerBottomEncoder.setPosition(0); 
  }

  public double conveyerTopMotorPosition(){
    return conveyerTopEncoder.getPosition(); 
  }

  public double conveyerTopMotorVelocity(){
    return conveyerTopEncoder.getVelocity(); 
  }

  public double conveyerTopMotorCurrent(){
    return conveyerTopMotor.getOutputCurrent(); 
  }

  public double conveyerBottomMotorPosition(){
    return conveyerBottomEncoder.getPosition(); 
  }

  public double conveyerBottomMotorVelocity(){
    return conveyerBottomEncoder.getVelocity(); 
  }

  public double conveyerBottomMotorCurrent(){
    return conveyerBottomMotor.getOutputCurrent(); 
  }


  public void setTopConveyerPIDF(double p, double i, double d, double f){
    conveyerTopMotor.getPIDController().setP(p); 
    conveyerTopMotor.getPIDController().setI(i); 
    conveyerTopMotor.getPIDController().setD(d); 
    conveyerTopMotor.getPIDController().setFF(f); 
  }

  public void setBottomConveyerPIDF(double p, double i, double d, double f){
    conveyerBottomMotor.getPIDController().setP(p); 
    conveyerBottomMotor.getPIDController().setI(i); 
    conveyerBottomMotor.getPIDController().setD(d); 
    conveyerBottomMotor.getPIDController().setFF(f); 
  }

  public void conveyerOutPutConstraints(double min, double max){
    conveyerTopMotor.getPIDController().setOutputRange(min, max); 
    conveyerBottomMotor.getPIDController().setOutputRange(min, max); 
  }

  public void setconveyerVelocityMode(){
    conveyerTopMotor.getPIDController().setReference(0, ControlType.kVelocity); 
    conveyerBottomMotor.getPIDController().setReference(0, ControlType.kVelocity); 
  }

  public void setconveyerPowerMode(){
    conveyerTopMotor.getPIDController().setReference(0, ControlType.kVoltage); 
    conveyerBottomMotor.getPIDController().setReference(0, ControlType.kVoltage); 
  }

  public void setVelocity(double conveyerVelocity){
    conveyerTopMotor.getPIDController().setReference(conveyerVelocity, ControlType.kVelocity); 
    conveyerBottomMotor.getPIDController().setReference(conveyerVelocity, ControlType.kVelocity); 
  }

  public void setConveyer(double conveyerSpeed){
    conveyerTopMotor.set(conveyerSpeed);
    conveyerBottomMotor.set(conveyerSpeed); 
  }

  public void stop(){
    conveyerTopMotor.stopMotor();
    conveyerBottomMotor.stopMotor();
  }
}

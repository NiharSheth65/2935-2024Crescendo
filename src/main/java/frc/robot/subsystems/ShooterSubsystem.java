// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private CANSparkMax topShooterMotor = new CANSparkMax(ShooterConstants.topShooterMotorId, MotorType.kBrushless); 
  private CANSparkMax bottomShooterMotor = new CANSparkMax(ShooterConstants.bottomShooterMotorId, MotorType.kBrushless);

  private RelativeEncoder topShooterEncoder = topShooterMotor.getEncoder(); 
  private RelativeEncoder bottomShooterEncoder = bottomShooterMotor.getEncoder(); 

  public ShooterSubsystem() {

    topShooterMotor.restoreFactoryDefaults(); 
    bottomShooterMotor.restoreFactoryDefaults(); 

    // topShooterMotor.setInverted(true);

    topShooterEncoder.setPosition(0); 
    bottomShooterEncoder.setPosition(0); 

    setRampRate(3);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Shuffleboard.getTab("shooter");


    // SmartDashboard.putNumber("velocity", leftShooterEncoder.getVelocity());  
  }

  public void setBrakeMode(){
    topShooterMotor.setIdleMode(IdleMode.kBrake); 
    bottomShooterMotor.setIdleMode(IdleMode.kBrake); 
  }

  public void setCoastMode(){
    topShooterMotor.setIdleMode(IdleMode.kCoast); 
    bottomShooterMotor.setIdleMode(IdleMode.kCoast); 
  }

  public void resetEncoders(){
    topShooterEncoder.setPosition(0); 
    bottomShooterEncoder.setPosition(0); 
  }

  public double topMotorPosition(){
    return topShooterEncoder.getPosition(); 
  }

  public double bottomMotorPosition(){
    return bottomShooterEncoder.getPosition(); 
  }

  public double topMotorVelocity(){
    return topShooterEncoder.getVelocity(); 
  }

  public double bottomMotorVelocity(){
    return bottomShooterEncoder.getVelocity(); 
  }

  public void setTopPIDF(double p, double i, double d, double f){
    topShooterMotor.getPIDController().setP(p); 
    topShooterMotor.getPIDController().setI(i); 
    topShooterMotor.getPIDController().setD(d); 
    topShooterMotor.getPIDController().setFF(f); 
  }

  public void setBottomPIDF(double p, double i, double d, double f){
    bottomShooterMotor.getPIDController().setP(p);
    bottomShooterMotor.getPIDController().setI(i);
    bottomShooterMotor.getPIDController().setD(d);
    bottomShooterMotor.getPIDController().setFF(f);
  }

  public void setTopMotorOutputConstraints(double min, double max){
    topShooterMotor.getPIDController().setOutputRange(min, max); 
  }

  public void setBottomMotorOutputConstraints(double min, double max){
    bottomShooterMotor.getPIDController().setOutputRange(min, max); 
  }

  public void setShooterVelocityMode(){
    topShooterMotor.getPIDController().setReference(0, ControlType.kVelocity); 
    bottomShooterMotor.getPIDController().setReference(0, ControlType.kVelocity); 
  }

  public void setShooterPowerMode(){
    topShooterMotor.getPIDController().setReference(0, ControlType.kCurrent); 
    bottomShooterMotor.getPIDController().setReference(0, ControlType.kCurrent); 
  }

  public void setVelocityTop(double topVelocity){
    topShooterMotor.getPIDController().setReference(topVelocity, ControlType.kVelocity); 
  }

  public void setVelocityBottom(double bottomVelocity){
    bottomShooterMotor.getPIDController().setReference(bottomVelocity, ControlType.kVelocity); 
  }

  public void setRampRate(double ramp){
    topShooterMotor.setClosedLoopRampRate(ramp);
    bottomShooterMotor.setClosedLoopRampRate(ramp);
  }

  public void setShooter(double topSpeed, double bottomSpeed){
    topShooterMotor.set(topSpeed);
    bottomShooterMotor.set(-bottomSpeed);
  }

  public void stop(){
    bottomShooterMotor.stopMotor();
    topShooterMotor.stopMotor();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase {

  private CANSparkMax wristMotor = new CANSparkMax(WristConstants.wristMotorId, MotorType.kBrushless);
  private RelativeEncoder wristEncoder = wristMotor.getEncoder();

  /** Creates a new WristSubsystem. */
  public WristSubsystem() {

    wristMotor.restoreFactoryDefaults();
    wristMotor.setInverted(true);
    wristEncoder.setPosition(0);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("wrist position", wristMotorPosition());   
    SmartDashboard.putNumber("wrist velocity", wristMotorVelocity()); 
  }

  public double wristMotorPosition(){
    return wristEncoder.getPosition(); 
  }

  public double wristMotorVelocity(){
    return wristEncoder.getVelocity(); 
  }

  public void resetEncoders(){
    wristEncoder.setPosition(0); 
  }

  public void setCoastMode(){
    wristMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setBrakeMode(){
    wristMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setWrist(double wristSpeed){
    wristMotor.set(wristSpeed);
  }

  public void stop(){
    wristMotor.stopMotor();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  private CANSparkMax leftFrontShooterMotor; 
  private CANSparkMax leftBackShooterMotor; 

  private CANSparkMax rightFrontShooterMotor;  
  private CANSparkMax rightBackShooterMotor; 

  // private RelativeEncoder leftShooterEncoder; 
  private RelativeEncoder rightShooterEncoder; 

  public ShooterSubsystem() {

    leftFrontShooterMotor = new CANSparkMax(ShooterConstants.leftFrontShooterMotorId, MotorType.kBrushless); 
    // leftBackShooterMotor = new CANSparkMax(ShooterConstants.leftBackShooterMotorId, MotorType.kBrushless); 

    rightFrontShooterMotor = new CANSparkMax(ShooterConstants.rightFrontShooterMotorId, MotorType.kBrushless);
    // rightBackShooterMotor = new CANSparkMax(ShooterConstants.rightBackShooterMotorId, MotorType.kBrushless);

    leftFrontShooterMotor.restoreFactoryDefaults(); 
    rightFrontShooterMotor.restoreFactoryDefaults(); 
  
    // leftBackShooterMotor.follow(leftFrontShooterMotor); 
    // rightBackShooterMotor.follow(rightFrontShooterMotor); 
    
    // rightFrontShooterMotor.setInverted(true);
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("shooter called", "true");
    // SmartDashboard.putNumber("velocity", leftShooterEncoder.getVelocity());  
  }

  public void setShooter(double topSpeed, double bottomSpeed){
    rightFrontShooterMotor.set(topSpeed);
    leftFrontShooterMotor.set(-bottomSpeed);
  }

  public void stop(){
    leftFrontShooterMotor.stopMotor();
    rightFrontShooterMotor.stopMotor();
  }
}

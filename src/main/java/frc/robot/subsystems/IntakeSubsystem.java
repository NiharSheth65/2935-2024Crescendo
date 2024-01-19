// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private CANSparkMax intakeMotor; 
  
  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax(0, MotorType.kBrushless); 
    intakeMotor.restoreFactoryDefaults(); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void setIntake(double speed){
    intakeMotor.set(speed);
  }

  public void stop(){
    intakeMotor.stopMotor();
  }
}

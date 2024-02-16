// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wristCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.WristSubsystem;

public class wristSetPosition extends Command {

  private WristSubsystem WRIST_SUBSYSTEM; 
  private PIDController wristPIDController; 

  double wristSetpoint; 
  double currentPosition; 

  /** Creates a new wristSetPosition. */
  public wristSetPosition(WristSubsystem wrist, double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.WRIST_SUBSYSTEM = wrist; 
    this.wristPIDController = new PIDController(WristConstants.wristKP, WristConstants.wristKI, WristConstants.wristKD); 
    this.wristSetpoint = setPoint; 
    addRequirements(WRIST_SUBSYSTEM);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wristPIDController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPosition = WRIST_SUBSYSTEM.wristMotorPosition(); 
    double wristSetValue = wristPIDController.calculate(currentPosition, wristSetpoint);
    
    if(wristSetValue >= WristConstants.wristMaxSpeed){
      wristSetValue = WristConstants.wristMaxSpeed; 
    }

    else if(wristSetValue <= -WristConstants.wristMaxSpeed){
      wristSetValue = -WristConstants.wristMaxSpeed; 
    }

    SmartDashboard.putNumber("wrist output", wristSetValue); 
  

    WRIST_SUBSYSTEM.setWrist(wristSetValue*WristConstants.wristSpeedTolerance);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    WRIST_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(Math.abs(WRIST_SUBSYSTEM.wristMotorPosition() - wristSetpoint) <  WristConstants.wristTolerance){
    //   return true; 
    // }

    // else{
    //   return false; 
    // }

    return false; 
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class autoDriveForwardSetDistance extends Command {
  /** Creates a new autoDriveForwardSetDistance. */

  private DriveSubsystem DRIVE_SUBSYSTEM; 
  private double distanceInInches;

  private PIDController leftWheelPowerPIDController; 
  private PIDController rightWheelPowerPIDController; 

  private SlewRateLimiter left_Limiter = new SlewRateLimiter(DriveConstants.driveSlew); 
  private SlewRateLimiter right_Limiter = new SlewRateLimiter(DriveConstants.driveSlew); 


  public autoDriveForwardSetDistance(DriveSubsystem drive, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.DRIVE_SUBSYSTEM = drive; 
    this.distanceInInches = distance;    

    this.leftWheelPowerPIDController = new PIDController(0.1, 0.06, 0); 
    this.rightWheelPowerPIDController = new PIDController(0.1, 0.06, 0); 
    
    // this.leftWheelPowerPIDController = new PIDController(0.1, 0, 0); 
    // this.rightWheelPowerPIDController = new PIDController(0.1, 0, 0); 


    
    addRequirements(DRIVE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DRIVE_SUBSYSTEM.resetEncoders();
    DRIVE_SUBSYSTEM.zeroHeading();
    leftWheelPowerPIDController.reset();
    rightWheelPowerPIDController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftWheelOutput = leftWheelPowerPIDController.calculate(DRIVE_SUBSYSTEM.leftEncoderToInches(), distanceInInches); 
    double rightWheelOutput = rightWheelPowerPIDController.calculate(DRIVE_SUBSYSTEM.rightEncoderToInches(), distanceInInches); 
  

    if(leftWheelOutput > 1.0){
      leftWheelOutput = 1.0; 
    }

    else{
      leftWheelOutput = leftWheelOutput;  
    }

    if(rightWheelOutput > 1.0){
      rightWheelOutput = 1.0;
    }

    else{
      rightWheelOutput = rightWheelOutput; 

    }
    DRIVE_SUBSYSTEM.setTank(left_Limiter.calculate(leftWheelOutput), right_Limiter.calculate(rightWheelOutput));
    
    SmartDashboard.putNumber("right output", rightWheelOutput); 
    SmartDashboard.putNumber("left output", leftWheelOutput); 
    SmartDashboard.putNumber("error", (distanceInInches - DRIVE_SUBSYSTEM.averageEncoderDistanceInInches()));
    
    // DRIVE_SUBSYSTEM.set(0.50, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DRIVE_SUBSYSTEM.stop(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // if(Math.abs(distanceInInches - DRIVE_SUBSYSTEM.averageEncoderDistanceInInches()) < 0.50){
      // return true; 
    // }

    // else{
      // return false; 
    // }

    // if(DRIVE_SUBSYSTEM.averageEncoderDistanceInInches() >= 10){
    //   return true; 
    // }
    // else{
    //   return false;    
    // }

    return false; 
  }
}

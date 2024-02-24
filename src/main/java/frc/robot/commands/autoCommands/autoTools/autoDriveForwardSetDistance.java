// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands.autoTools;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.drivetrainCommands.DriveVelocityControl;
import frc.robot.subsystems.DriveSubsystem;

public class autoDriveForwardSetDistance extends Command {
  /** Creates a new autoDriveForwardSetDistance. */

  private DriveSubsystem DRIVE_SUBSYSTEM; 
  private double distanceInInches;

  private PIDController leftWheelPowerPIDController; 
  private PIDController rightWheelPowerPIDController; 
  private PIDController gyroPIDController; 
  
  double gryoTarget; 


  private boolean driveCompleted; 

  private SlewRateLimiter left_Limiter = new SlewRateLimiter(10); 
  private SlewRateLimiter right_Limiter = new SlewRateLimiter(10); 

  double maxDriveSpeed; 


  public autoDriveForwardSetDistance(DriveSubsystem drive, double distance, double maxSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.DRIVE_SUBSYSTEM = drive; 
    this.distanceInInches = distance;    

    this.leftWheelPowerPIDController = new PIDController(0.075, 0.01, 0); 
    this.rightWheelPowerPIDController = new PIDController(0.075, 0.01, 0); 
    this.gyroPIDController = new PIDController(0.005, 0, 0); 

    this.maxDriveSpeed = maxSpeed; 
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
    gyroPIDController.reset();

    gryoTarget = DRIVE_SUBSYSTEM.getYaw(); 

    driveCompleted = false; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double leftWheelOutput = leftWheelPowerPIDController.calculate(DRIVE_SUBSYSTEM.leftEncoderToInches(), distanceInInches); 
    double rightWheelOutput = rightWheelPowerPIDController.calculate(DRIVE_SUBSYSTEM.rightEncoderToInches(), distanceInInches); 
    double gyroSpeed = gyroPIDController.calculate(DRIVE_SUBSYSTEM.getYaw(), gryoTarget); 

    if(leftWheelOutput > maxDriveSpeed){
      leftWheelOutput = maxDriveSpeed; 
    }

    else if(leftWheelOutput < -maxDriveSpeed){
      leftWheelOutput = -maxDriveSpeed; 
    }

    else{
      leftWheelOutput = leftWheelOutput;  
    }

    if(rightWheelOutput > maxDriveSpeed){
      rightWheelOutput = maxDriveSpeed;
    }

    else if(rightWheelOutput < -maxDriveSpeed){
      rightWheelOutput = -maxDriveSpeed;
    }

    else{
      rightWheelOutput = rightWheelOutput; 

    }
    DRIVE_SUBSYSTEM.setTank(left_Limiter.calculate(leftWheelOutput + gyroSpeed), right_Limiter.calculate(rightWheelOutput - gyroSpeed));
    
    if(Math.abs(distanceInInches - DRIVE_SUBSYSTEM.averageEncoderDistanceInInches()) < 2){
      driveCompleted = true; 
    }

    else{
      driveCompleted = false; 
    }

    SmartDashboard.putNumber("right output", rightWheelOutput); 
    SmartDashboard.putNumber("left output", leftWheelOutput); 
    SmartDashboard.putNumber("error", (distanceInInches - DRIVE_SUBSYSTEM.averageEncoderDistanceInInches()));
    SmartDashboard.putBoolean("drive complete", driveCompleted); 
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

    if(driveCompleted == true){
      return true; 
    }

    return false; 
  }
}

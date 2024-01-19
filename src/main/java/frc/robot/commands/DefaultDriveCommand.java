// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends Command {
  
  private DriveSubsystem DRIVE_SUBSYSTEM; 
  private Joystick joy; 

  double drive; 
  double turn; 
  double driveSpeed; 

  SlewRateLimiter drive_Limiter = new SlewRateLimiter(DriveConstants.driveSlew); 
  SlewRateLimiter turn_Limiter = new SlewRateLimiter(DriveConstants.turnSlew); 


  
  /** Creates a new DefaultDriveCommand. */
  public DefaultDriveCommand(DriveSubsystem drive, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.DRIVE_SUBSYSTEM = drive; 
    this.joy = joystick; 

    addRequirements(DRIVE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive = 0; 
    turn = 0;

    DRIVE_SUBSYSTEM.zeroHeading();
    DRIVE_SUBSYSTEM.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(joy.getRawButton(OperatorConstants.BUTTON_RB_PORT)){
      driveSpeed = DriveConstants.driveFastSpeed; 
    }

    else{
      driveSpeed = DriveConstants.driveSlowSpeed; 
    }

    if(Math.abs(joy.getRawAxis(OperatorConstants.driveJoystickAxis)) < DriveConstants.driveDeadband){
      drive = 0; 
    }

    else{
      drive = -joy.getRawAxis(OperatorConstants.driveJoystickAxis) * driveSpeed; 
    }

    if(Math.abs(joy.getRawAxis(OperatorConstants.turnJoystickAxis)) < DriveConstants.turnDead){
      turn = 0;
    }

    else{
      turn = -joy.getRawAxis(OperatorConstants.turnJoystickAxis)*DriveConstants.turnSpeed; 
    }
    
    DRIVE_SUBSYSTEM.set(drive_Limiter.calculate(drive), turn_Limiter.calculate(turn));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DRIVE_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

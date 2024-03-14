// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrainCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTurnCommand extends Command {

  private DriveSubsystem DRIVE_SUBSYSTEM; 

  private double tSlew = DriveConstants.turnSlew; 
  private SlewRateLimiter turn_Limiter = new SlewRateLimiter(tSlew); 

  private double turnDirection; 

  private double leftMultiplier; 
  private double rightMultiplier; 

  private boolean end; 

  /** Creates a new DriveTurnCommand. */
  public DriveTurnCommand(DriveSubsystem drive, double direction, boolean endCommand) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.DRIVE_SUBSYSTEM = drive; 
    this.turnDirection = direction; 
    this.end = endCommand; 
    addRequirements(drive);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DRIVE_SUBSYSTEM.setRightPowerMode();
    DRIVE_SUBSYSTEM.setLeftPowerMode();

    rightMultiplier = 0; 
    leftMultiplier = 0; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(turnDirection == 1){
      leftMultiplier = 1; 
      rightMultiplier = -1; 
    }

    else if(turnDirection == 0){
      leftMultiplier = -1; 
      rightMultiplier = 1; 
    }

    else{
      leftMultiplier = 0; 
      rightMultiplier = 0;  
    }

    DRIVE_SUBSYSTEM.setTank(leftMultiplier*1, rightMultiplier*1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(end == true){
      return true; 
    }

    else{
      return false;
    }

  }
}

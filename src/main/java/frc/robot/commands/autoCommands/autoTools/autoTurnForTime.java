// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands.autoTools;

import java.util.concurrent.TransferQueue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class autoTurnForTime extends Command {

  private DriveSubsystem DRIVE_SUBSYSTEM; 

  private double runTime; 
  private double turnDirection; 

  private double initTime; 

  private double leftMultiplier; 
  private double rightMultiplier; 

  /** Creates a new autoTurnForTime. */
  public autoTurnForTime(DriveSubsystem drive, double time, double direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.DRIVE_SUBSYSTEM = drive; 
    this.runTime = time; 
    this.turnDirection = direction; 
    addRequirements(DRIVE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTime = System.currentTimeMillis(); 
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

      DRIVE_SUBSYSTEM.setTank(leftMultiplier*0.5, rightMultiplier*0.5);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(System.currentTimeMillis() - initTime) > runTime){
      return true; 
    }

    else{
      return false;
    }
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands.autoTools;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class autoTurnOnHeadingCommand extends Command {
  
  private final DriveSubsystem DRIVE_SUBSYSTEM; 
  private final double heading; 

  private PIDController turnController; 

  private boolean turnCompleted = false; 
  private int turnCompletedCounter = 0; 
  private double timeAtTurn; 

  /** Creates a new autoTurnOnHeadingCommand. */
  public autoTurnOnHeadingCommand(DriveSubsystem drive, double heading) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.DRIVE_SUBSYSTEM = drive; 
    this.heading = heading; 

    this.turnController = new PIDController(0.0185, 0.01, 0); 
    
    addRequirements(DRIVE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
    
  
  @Override
  public void initialize() {
    turnController.reset();
    turnCompleted = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    DRIVE_SUBSYSTEM.getHeadingError(heading); 
    double turnSpeed = turnController.calculate(DRIVE_SUBSYSTEM.getHeadingError(heading));     

    
    if(DRIVE_SUBSYSTEM.getHeadingError(heading) < 1.00){
      turnCompleted = true;  
    }

    else{
      turnCompleted = false;  
    }


    
  

    // SmartDashboard.putNumber("heading error", DRIVE_SUBSYSTEM.getHeadingError(heading));
    // SmartDashboard.putNumber("adjustment speed", turnSpeed);
    // SmartDashboard.putNumber("time at turn", timeAtTurn); 
    SmartDashboard.putBoolean("turn completed", turnCompleted);
    SmartDashboard.putNumber("turn counter", turnCompletedCounter); 

    if(turnSpeed > 1.0){
      turnSpeed = 1.0; 
    }

    else if(turnSpeed < -1.0){
      turnSpeed = -1.0; 
    }

    DRIVE_SUBSYSTEM.setTank(-turnSpeed, turnSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DRIVE_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if(turnCompleted == true){
      return true; 
    }
    
    else{
      return false;
    }
  }
}

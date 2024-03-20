// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands.autoTools;

import javax.swing.plaf.basic.BasicInternalFrameTitlePane.SystemMenuBar;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.autoConstants;
import frc.robot.Constants.photonVisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class autoTurnCommand extends Command {
  /** Creates a new autoTurnCommand. */

  private DriveSubsystem DRIVE_SUBSYSTEM; 
  private double desiredAngle; 

  private boolean endCommand; 
  private PIDController turnController; 

  private SlewRateLimiter turnLimiter = new SlewRateLimiter(DriveConstants.turnSlew); 

  private double initTime; 

  public autoTurnCommand(DriveSubsystem drive, double angle, boolean end) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.DRIVE_SUBSYSTEM = drive; 
    this.desiredAngle = angle; 
    this.turnController = new PIDController(photonVisionConstants.turnKp, photonVisionConstants.turnKi, photonVisionConstants.turnKd); 
    this.endCommand = end; 

    addRequirements(DRIVE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnController.reset();
    DRIVE_SUBSYSTEM.zeroHeading();
    initTime = System.currentTimeMillis(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double outputSpeed = turnController.calculate(DRIVE_SUBSYSTEM.getYaw(), desiredAngle); 

    if(outputSpeed > autoConstants.gyroTurnMaxSpeed){
      outputSpeed = autoConstants.gyroTurnMaxSpeed; 
    }

    else if(outputSpeed < -autoConstants.gyroTurnMaxSpeed){
      outputSpeed = -autoConstants.gyroTurnMaxSpeed; 
    }

    else{
      outputSpeed = outputSpeed;
    }
    DRIVE_SUBSYSTEM.setTank(-turnLimiter.calculate(outputSpeed), turnLimiter.calculate(outputSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DRIVE_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(endCommand){
      return true; 
    }

    else if(Math.abs(initTime - System.currentTimeMillis()) > 2000){
      return true;
    }

    else if(Math.abs(DRIVE_SUBSYSTEM.getYaw() - desiredAngle) < 10){
      return true; 
    }
    
    else{
      return false;
    }
  }
}

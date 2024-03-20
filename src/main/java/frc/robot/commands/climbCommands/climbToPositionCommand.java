// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climbCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class climbToPositionCommand extends Command {

    
  private ClimbSubsystem CLIMB_SUBSYSTEM; 
  private PIDController pidController; 
  
  double climbSetpoint; 
  double measurement; 


  /** Creates a new climbToPositionCommand. */
  public climbToPositionCommand(ClimbSubsystem climb, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.CLIMB_SUBSYSTEM = climb; 
    this.pidController = new PIDController(ClimbConstants.climbKp, ClimbConstants.climbKi, ClimbConstants.climbKd); 
    this.climbSetpoint = setpoint; 

    addRequirements(CLIMB_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double climbEncoderPosition = CLIMB_SUBSYSTEM.climbMotorPosition(); 
    double climbSpeed = pidController.calculate(climbEncoderPosition, climbSetpoint); 

    if(climbSpeed > ClimbConstants.climbMaxSpeed){
      climbSpeed = ClimbConstants.climbMaxSpeed; 
    }

    else if(climbSpeed < -ClimbConstants.climbMaxSpeed){
      climbSpeed = -ClimbConstants.climbMaxSpeed;
    }

    CLIMB_SUBSYSTEM.setClimb(climbSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CLIMB_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(Math.abs(CLIMB_SUBSYSTEM.climbMotorPosition() - climbSetpoint) < 2){
      return true;
    }

    else{
      return false; 
    }
    
  }
}

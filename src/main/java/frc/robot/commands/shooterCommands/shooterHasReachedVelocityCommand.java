// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class shooterHasReachedVelocityCommand extends Command {

  private LightSubsystem LIGHT_SUBSYSTEM; 
  private ShooterSubsystem SHOOTER_SUBSYSTEM;
  
  private double bottomTargetVelocity; 
  private double topTargetVelocity; 

  private boolean endCommand; 

  private int red, green, blue; 

  private double shooterBottomPresentVelocity; 
  private double shooterTopPresentVelocity; 

  private boolean hasReached; 

  private double initTime; 


  /** Creates a new shooterHasReachedVelocityCommand. */
  public shooterHasReachedVelocityCommand(LightSubsystem led, ShooterSubsystem shooter, double targetTopVelocity, double targetBottomVelocity, boolean end) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.LIGHT_SUBSYSTEM = led; 
    this.SHOOTER_SUBSYSTEM = shooter; 
    this.topTargetVelocity = targetTopVelocity; 
    this.bottomTargetVelocity = targetBottomVelocity; 
    this.endCommand = end; 

    addRequirements(LIGHT_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterBottomPresentVelocity = 0; 
    shooterTopPresentVelocity = 0; 
    hasReached = false;
    initTime = System.currentTimeMillis(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterTopPresentVelocity = SHOOTER_SUBSYSTEM.bottomMotorVelocity(); 
    shooterBottomPresentVelocity = SHOOTER_SUBSYSTEM.topMotorVelocity(); 

    if((shooterTopPresentVelocity/ (bottomTargetVelocity)) > 0.90 && (shooterBottomPresentVelocity/ (topTargetVelocity)) > 0.90){
      red = 0;  
      green = 255; 
      blue = 0; 
      hasReached = true; 
    }

    else{
      red = 200;  
      green = 0; 
      blue = 200; 
      hasReached = false; 
    }

    LIGHT_SUBSYSTEM.setOneColour(red, blue, green);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(endCommand == true){
      return true; 
    }

    else if(hasReached == true){
      return true; 
    }

    else if(Math.abs(System.currentTimeMillis() - initTime) > 3000){
      return true; 
    }

    else{
      return false; 
    }
  }
}

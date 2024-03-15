// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ledCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ledRevedUpCommand extends Command {

  private LightSubsystem LIGHT_SUBSYSTEM; 
  private ShooterSubsystem SHOOTER_SUBSYSTEM;
  
  private double bottomTargetVelocity; 
  private double topTargetVelocity; 

  private boolean endCommand; 

  private int red, green, blue; 

  /** Creates a new ledRevedUpCommand. */
  public ledRevedUpCommand(LightSubsystem led, ShooterSubsystem shooter, double targetTopVelocity, double targetBottomVelocity, boolean end) {
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Math.abs(SHOOTER_SUBSYSTEM.bottomMotorVelocity() - bottomTargetVelocity) < 100){
      red = 0;  
      green = 255; 
      blue = 0; 
    }

    LIGHT_SUBSYSTEM.setOneColour(red, green, blue);
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

    else{
      return false; 
    }
  }
}

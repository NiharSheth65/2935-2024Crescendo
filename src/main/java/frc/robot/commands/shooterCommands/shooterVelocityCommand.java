// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class shooterVelocityCommand extends Command {
  
  private ShooterSubsystem SHOOTER_SUBSYSTEM; 

  private double topShooterSpeed; 
  private double bottomShooterSpeed; 

  private double topShooterKp = 6e-5; 
  private double topShooterKi = 0.0000; 
  private double topShooterKd = 0; 
  private double topShooterKIz = 6e-5; 
  private double topShooterKFf = 0.000015;
  
  private double topMax = 1; 
  private double topBottom = -1; 
  
  /** Creates a new shooterVelocityCommand. */
  public shooterVelocityCommand(ShooterSubsystem shooter, double topMotorSpeed, double bottomMotorSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.SHOOTER_SUBSYSTEM = shooter; 
    this.topShooterSpeed = topMotorSpeed; 
    this.bottomShooterSpeed = bottomMotorSpeed; 
    addRequirements(SHOOTER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooterCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  
  private double bottomShooterKp = 6e-5; 
  private double bottomShooterKi = 0.0000; 
  private double bottomShooterKd = 0; 
  private double bottomShooterKIz = 6e-5; 
  private double bottomShooterKFf = 0.000015;
  
  private double topMax = 1; 
  private double topMin = -1; 

  private double bottomMax = 1; 
  private double bottomMin = -1; 

  private SlewRateLimiter topLimiter = new SlewRateLimiter(2); 
  private SlewRateLimiter bottomLimiter = new SlewRateLimiter(2); 

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
  public void initialize() {

    // topShooterSpeed = 0; 
    // bottomShooterSpeed = 0; 
    SHOOTER_SUBSYSTEM.setShooterVelocityMode();
    SHOOTER_SUBSYSTEM.setRampRate(1);
    SHOOTER_SUBSYSTEM.setTopPIDF(topShooterKp, topShooterKi, topShooterKd, topShooterKFf);
    SHOOTER_SUBSYSTEM.setBottomPIDF(bottomShooterKp, bottomShooterKi, bottomShooterKd, bottomShooterKFf);

    SHOOTER_SUBSYSTEM.setTopMotorOutputConstraints(topMin, topMax);
    SHOOTER_SUBSYSTEM.setBottomMotorOutputConstraints(bottomMin, bottomMax);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SHOOTER_SUBSYSTEM.setVelocityTop((topShooterSpeed*3.0));
    SHOOTER_SUBSYSTEM.setVelocityBottom((bottomShooterSpeed*3.0));

    SmartDashboard.putNumber("top motor veocity", SHOOTER_SUBSYSTEM.topMotorVelocity()); 
    SmartDashboard.putNumber("bottom motor veocity", SHOOTER_SUBSYSTEM.bottomMotorVelocity()); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SHOOTER_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

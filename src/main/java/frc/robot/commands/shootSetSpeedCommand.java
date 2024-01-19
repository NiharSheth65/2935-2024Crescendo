// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class shootSetSpeedCommand extends Command {
  /** Creates a new shootSetSpeedCommand. */

  private ShooterSubsystem SHOOTER_SUBSYSTEM; 

  double topSpeed;
  double bottomSpeed;  

  public shootSetSpeedCommand(ShooterSubsystem shooter, double speedT, double speedB) {

    this.SHOOTER_SUBSYSTEM = shooter; 
    this.topSpeed = speedT;  
    this.bottomSpeed = speedB;  
    addRequirements(SHOOTER_SUBSYSTEM);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // shootSpeed = 0; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    // SmartDashboard.putNumber("shooter speed", shootSpeed); 
    SHOOTER_SUBSYSTEM.setShooter(topSpeed, bottomSpeed);
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

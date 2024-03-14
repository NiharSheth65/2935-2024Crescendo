// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands.autoTools;

import edu.wpi.first.wpilibj2.command.Command;

public class autoDelayCommand extends Command {
  /** Creates a new autoDelayCommand. */


  private double dt; 
  private double initialTime; 

  public autoDelayCommand(double delayTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.dt = delayTime; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialTime = System.currentTimeMillis(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if(System.currentTimeMillis() - initialTime > dt){
      return true; 
    }

    else{
      return false; 
    }
  }
}

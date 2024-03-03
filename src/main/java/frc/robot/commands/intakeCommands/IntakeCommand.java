// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
  /** Creates a new IntakeCommand. */

  private IntakeSubsystem INTAKE_SUBSYSTEM; 
  
  double intakeSpeed; 

  double initTime; 

  boolean endCommand; 

  public IntakeCommand(IntakeSubsystem intake, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.INTAKE_SUBSYSTEM = intake; 
    this.intakeSpeed = speed;
    addRequirements(INTAKE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTime = System.currentTimeMillis(); 
    endCommand = false; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    if(INTAKE_SUBSYSTEM.hasIntaked() == true && Math.abs(System.currentTimeMillis() - initTime) < 500){
      endCommand = false; 
    }

    else if(INTAKE_SUBSYSTEM.hasIntaked() == true && Math.abs(System.currentTimeMillis() - initTime) > 500){
      endCommand = true;  
    }

    else{
      endCommand = false; 
    }

    SmartDashboard.putBoolean("end command", endCommand); 

    INTAKE_SUBSYSTEM.setIntake(intakeSpeed, intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    INTAKE_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    // if(endCommand == true){
    //   return true; 
    // }

    // else{
    //   return false; 
    // }

    return false; 
    
  }
}

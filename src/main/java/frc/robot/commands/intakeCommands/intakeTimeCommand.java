// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class intakeTimeCommand extends Command {
  /** Creates a new intakeTimeCommand. */
  private IntakeSubsystem INTAKE_SUBSYSTEM; 
  
  double intakeSpeed; 
  
  double initialTime;

  double delayTime; 
  double runTime; 


  double runSpeed; 

  boolean commandEnded = false; 

  public intakeTimeCommand(IntakeSubsystem intake, double speed, double runningTime, double delayingTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.INTAKE_SUBSYSTEM = intake; 
    this.intakeSpeed = speed;
    this.delayTime = delayingTime; 
    this.runTime = runningTime; 
    addRequirements(INTAKE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialTime = System.currentTimeMillis(); 
    commandEnded = false; 
    runSpeed = 0; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Math.abs(System.currentTimeMillis() - initialTime) < delayTime){
      runSpeed = 0; 
    }

    else{
      runSpeed = intakeSpeed; 
    }

    INTAKE_SUBSYSTEM.setIntake(runSpeed, runSpeed);


    SmartDashboard.putBoolean("intake command ended", commandEnded); 
    SmartDashboard.putNumber("runSpeed", runSpeed); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    INTAKE_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if(Math.abs(System.currentTimeMillis() - initialTime) > runTime + delayTime){
      return true; 
    }
   
    else{
      return false;
    }


  }
}

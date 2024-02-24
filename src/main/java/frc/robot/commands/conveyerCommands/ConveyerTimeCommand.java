// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.conveyerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyerSubsystem;

public class ConveyerTimeCommand extends Command {


  private  ConveyerSubsystem CONVEYER_SUBSYSTEM; 
  private double conveyerSpeed;
  private double conveyerRunTime; 
  private double initTime; 
  
  /** Creates a new ConveyerTimeCommand. */
  public ConveyerTimeCommand(ConveyerSubsystem conveyer, double speed, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.conveyerSpeed = speed; 
    this.CONVEYER_SUBSYSTEM = conveyer; 
    this.conveyerRunTime = time; 
    addRequirements(CONVEYER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTime = System.currentTimeMillis(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    CONVEYER_SUBSYSTEM.setConveyer(conveyerSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CONVEYER_SUBSYSTEM.stop(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if(Math.abs(initTime - System.currentTimeMillis()) > conveyerRunTime){
      return true; 
    }

    else{
      return false;
    }
  }
}

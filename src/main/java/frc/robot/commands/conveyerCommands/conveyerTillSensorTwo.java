// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.conveyerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.conveyerConstants;
import frc.robot.subsystems.ConveyerSubsystem;

public class conveyerTillSensorTwo extends Command {

  private ConveyerSubsystem CONVEYER_SUBSYSTEM; 

  private double initTime; 
  private double initTimeInitTime; 


  /** Creates a new conveyerTillSensorTwo. */
  public conveyerTillSensorTwo(ConveyerSubsystem conveyer) {
    // Use addRequirements() here to declare subsystem dependencies.\
       // Use addRequirements() here to declare subsystem dependencies.
    this.CONVEYER_SUBSYSTEM = conveyer; 
    addRequirements(CONVEYER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTimeInitTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    CONVEYER_SUBSYSTEM.setConveyer(conveyerConstants.conveyerInSpeed);

    if(CONVEYER_SUBSYSTEM.intakeSwitchTwoValue() == true || CONVEYER_SUBSYSTEM.intakeSwitchThreeValue() == true){
      initTime = System.currentTimeMillis(); 
    }

    else{
      initTime = initTimeInitTime; 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CONVEYER_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(CONVEYER_SUBSYSTEM.intakeSwitchTwoValue() == true){
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.conveyerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyerSubsystem;

public class conveyerEncoderCommand extends Command {

  private ConveyerSubsystem CONVEYER_SUBSYSTEM; 

  private double targetEncoderValue; 
  private double conveyerSpeed; 

  /** Creates a new conveyerEncoderCommand. */
  public conveyerEncoderCommand(ConveyerSubsystem conveyer, double encoderSetpoint, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.CONVEYER_SUBSYSTEM = conveyer; 
    this.targetEncoderValue = encoderSetpoint; 
    this.conveyerSpeed = speed; 
    addRequirements(CONVEYER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CONVEYER_SUBSYSTEM.resetEncoders();
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
    if(Math.abs(CONVEYER_SUBSYSTEM.conveyerTopMotorPosition() - targetEncoderValue) < 0.5){
      return true; 
    }

    else{
      return false; 
    }
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeCommands;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class intakeTillFirstSensor extends Command {
  /** Creates a new intakeTillFirstSensor. */

  private IntakeSubsystem INTAKE_SUBSYSTEM; 
  private ConveyerSubsystem CONVEYER_SUBSYSTEM; 

  private double initTime; 

  public intakeTillFirstSensor(IntakeSubsystem intake, ConveyerSubsystem conveyer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.INTAKE_SUBSYSTEM = intake;
    this.CONVEYER_SUBSYSTEM = conveyer; 
    addRequirements(INTAKE_SUBSYSTEM);
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
    INTAKE_SUBSYSTEM.setIntake(IntakeConstants.intakeSpeed, IntakeConstants.intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    INTAKE_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(CONVEYER_SUBSYSTEM.intakeSwitchOneValue() == true){
      return true; 
    }

    else if(Math.abs(System.currentTimeMillis() - initTime) > 3000){
      return true; 
    }

    else{
      return true; 
    }
  }
}

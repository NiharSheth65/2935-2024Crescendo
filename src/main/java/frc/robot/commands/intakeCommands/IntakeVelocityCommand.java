// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeVelocityCommand extends Command {

  private IntakeSubsystem intake; 
  private double intakeSpeed; 

  private double intakeMotorKp = 10; 
  private double intakeMotorKi = 0.000000; 
  private double intakeMotorKd = 0; 
  private double intakeMotorKIz = 0; 
  private double intakeMotorKFf = 0.000015; 

  private double intakeMax = 1; 
  private double intakeMin = -1; 

  /** Creates a new IntakeVelocityCommand. */
  public IntakeVelocityCommand(IntakeSubsystem intake, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake; 
    this.intakeSpeed = speed; 
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setIntakeVelocityMode();
    intake.setLeftIntakePIDF(intakeMotorKp, intakeMax, intakeMotorKd, intakeMotorKFf);
    intake.intakeOutPutConstraints(intakeMin, intakeMax);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setVelocity(intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

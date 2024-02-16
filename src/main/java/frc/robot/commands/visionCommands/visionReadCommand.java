// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.visionCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.VisionSubsystem;

public class visionReadCommand extends Command {

  private VisionSubsystem vision; 

  /** Creates a new visionReadCommand. */
  public visionReadCommand(VisionSubsystem vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.vision = vision; 
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // vision.setLED(0);
    vision.setPipeline(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double tx = vision.tx();
    double ty = vision.ty();
    double ta = vision.ta();
    double tv = vision.tv();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

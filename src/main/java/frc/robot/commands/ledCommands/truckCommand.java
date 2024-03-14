// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ledCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.TruckLightSubsystem;

public class truckCommand extends Command {
  /** Creates a new truckCommand. */

  TruckLightSubsystem TRUCK_LIGHT_SUBSYSTEM; 
  private double truckBrightness; 

  public truckCommand(TruckLightSubsystem truckLight, double brightness) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.TRUCK_LIGHT_SUBSYSTEM = truckLight;   
    this.truckBrightness = brightness; 

    addRequirements(TRUCK_LIGHT_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TRUCK_LIGHT_SUBSYSTEM.setTruckBrightness(truckBrightness);
  }

  // Called once the command end or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

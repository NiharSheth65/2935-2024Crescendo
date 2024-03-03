// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ledCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LedConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;

public class LedCommand extends Command {

  private LightSubsystem LIGHT_SUBSYSTEM; 
  private IntakeSubsystem INTAKE_SUBSYSTEM; 

  private int red, green, blue; 
  /** Creates a new LedCommand. */

  public LedCommand(LightSubsystem light, IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.LIGHT_SUBSYSTEM = light; 
    this.INTAKE_SUBSYSTEM = intake; 

    addRequirements(LIGHT_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LIGHT_SUBSYSTEM.setOneColour(0, 0, 0);
    red = 0; 
    green = 0; 
    blue = 0;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     if(INTAKE_SUBSYSTEM.hasIntaked() == true){
      red = LedConstants.orangeColourCode[0];
      green = LedConstants.orangeColourCode[1];
      blue = LedConstants.orangeColourCode[2];
      // LED_SUBSYSTEM.setOneColour(red, green, blue);
    }


    else if(INTAKE_SUBSYSTEM.hasIntaked() == false){
      red = LedConstants.blueColourCode[0]; 
      green = LedConstants.blueColourCode[1]; 
      blue = LedConstants.blueColourCode[2];
    }

    LIGHT_SUBSYSTEM.setOneColour(red, green, blue); 
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ledCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LedConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;

public class LedCommand extends Command {

  private RobotContainer m_robotContainer;

  private LightSubsystem LIGHT_SUBSYSTEM; 
  private IntakeSubsystem INTAKE_SUBSYSTEM; 
  private ConveyerSubsystem CONVEYER_SUBSYSTEM; 

  private int red, green, blue; 
  private int allianceNumber; 

  /** Creates a new LedCommand. */

  public LedCommand(LightSubsystem light, IntakeSubsystem intake, ConveyerSubsystem conveyer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.LIGHT_SUBSYSTEM = light; 
    this.INTAKE_SUBSYSTEM = intake; 
    this.CONVEYER_SUBSYSTEM = conveyer; 

    addRequirements(LIGHT_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LIGHT_SUBSYSTEM.setOneColour(0, 0, 0);
    red = 0; 
    green = 0; 
    blue = 0; 

    allianceNumber = m_robotContainer.alliance(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


     if(CONVEYER_SUBSYSTEM.intakeSwitchOneValue() == true || CONVEYER_SUBSYSTEM.intakeSwitchTwoValue() == true  || CONVEYER_SUBSYSTEM.intakeSwitchThreeValue() == true){
      red = LedConstants.whiteColourCode[0];
      green = LedConstants.whiteColourCode[1];
      blue = LedConstants.whiteColourCode[2];
      // LED_SUBSYSTEM.setOneColour(red, green, blue);
    }

    else{
        
      if(allianceNumber == 0){
        red = 255; 
        green = 0; 
        blue = 0; 
      }

      else if(allianceNumber == 1){
        red = 0;
        green = 0; 
        blue = 255; 
      }

      else{
        red = 255; 
        green = 0; 
        blue = 255; 
      }

    }


    // else if(INTAKE_SUBSYSTEM.hasIntaked() == false){
    //   red = LedConstants.blueColourCode[0]; 
    //   green = LedConstants.blueColourCode[2]; 
    //   blue = LedConstants.blueColourCode[1];
    // }

    LIGHT_SUBSYSTEM.setOneColour(red, blue, green); 
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommandBlocks;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.photonVisionConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.conveyerCommands.conveyerTillSensorCleared;
import frc.robot.commands.photonvisionCommands.photonVisionDriveAndAlignCommand;
import frc.robot.commands.shooterCommands.shooterHasReachedVelocityCommand;
import frc.robot.commands.shooterCommands.shooterVelocityCommand;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.PhotonvisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoShootNoteCommand extends SequentialCommandGroup {

  private RobotContainer m_RobotContainer; 
  private int allianceNumber = m_RobotContainer.alliance(); 
  private int tagId; 


  /** Creates a new autoShootNoteCommand. */
  public autoShootNoteCommand(DriveSubsystem drive, PhotonvisionSubsystem photon, LightSubsystem led, ConveyerSubsystem conveyer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if(allianceNumber == 0){
      tagId = photonVisionConstants.speakerMiddleRedID; 
    }

    else if(allianceNumber == 1){
      tagId = photonVisionConstants.speakerMiddleBlueID; 
    }

    else{
      tagId = 0; 
    }

    addCommands(

      new ParallelDeadlineGroup(

        new SequentialCommandGroup(                
          new photonVisionDriveAndAlignCommand(photon, drive, 0, 0, false, tagId), 
          // new shooterHasReachedVelocityCommand(led, shooter, ShooterConstants.speakerTopMotorSpeed, ShooterConstants.speakerBottomMotorSpeed, false), 
          new conveyerTillSensorCleared(conveyer)
        )
        // new shooterVelocityCommand(shooter, ShooterConstants.speakerTopMotorSpeed, ShooterConstants.speakerBottomMotorSpeed)
      
      )
  
    );
  }
}

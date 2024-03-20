// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommandBlocks;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.photonVisionConstants;
import frc.robot.commands.photonvisionCommands.photonVisionDriveAndAlignCommand;
import frc.robot.commands.photonvisionCommands.photonVisionDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonvisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoHuntTargetCommand extends SequentialCommandGroup {


  private RobotContainer m_RobotContainer; 
  private int allianceNumber = m_RobotContainer.alliance(); 
  private int tagId; 
  private double distanceToDrive; 

  /** Creates a new autoHuntTargetCommand. */
  public autoHuntTargetCommand(DriveSubsystem drive, PhotonvisionSubsystem photon) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // change to drive and align at the same time 

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
      new photonVisionDriveAndAlignCommand(photon, drive, 0, 0, false, photonVisionConstants.speakerMiddleBlueID), 
      new photonVisionDriveCommand(photon, drive, false, photonVisionConstants.speakerMiddleBlueID), 
      new photonVisionDriveAndAlignCommand(photon, drive, 0, 0, false, photonVisionConstants.speakerMiddleBlueID)
    );
  }
}

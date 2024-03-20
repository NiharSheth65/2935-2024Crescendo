// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands.autoNoteBlocks;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.autoCommandBlocks.autoDigestNoteCommand;
import frc.robot.commands.autoCommandBlocks.autoEatNoteCommand;
import frc.robot.commands.autoCommandBlocks.autoHuntNoteCommand;
import frc.robot.commands.autoCommandBlocks.autoHuntTargetCommand;
import frc.robot.commands.autoCommandBlocks.autoShootNoteCommand;
import frc.robot.commands.autoCommands.autoTools.autoDriveForwardSetDistance;
import frc.robot.commands.autoCommands.autoTools.autoTurnCommand;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.PhotonvisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class note4Block extends SequentialCommandGroup {
  /** Creates a new note4Block. */
  public note4Block(DriveSubsystem drive, VisionSubsystem vision, PhotonvisionSubsystem photon, IntakeSubsystem intake, ShooterSubsystem shooter, ConveyerSubsystem conveyer, LightSubsystem led, int aprilTagID) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new autoTurnCommand(drive, -30, false), 

      new autoHuntNoteCommand(drive, vision), 
      new autoEatNoteCommand(drive, conveyer, intake),

      new ParallelCommandGroup(

          new autoDigestNoteCommand(intake, shooter, conveyer),

          new SequentialCommandGroup(
              new autoTurnCommand(drive, -50, false), 
              new autoDriveForwardSetDistance(drive, -25, DriveConstants.autoDriveSpeed),
              new autoTurnCommand(drive, 90, false), 
              new autoHuntTargetCommand(drive, photon) 
          )

      ), 
     
      new autoShootNoteCommand(drive, photon, led, shooter, conveyer)
    );
  }
}

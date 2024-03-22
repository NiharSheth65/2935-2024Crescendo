// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands.autoNoteBlocks;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autoCommandBlocks.autoDigestNoteCommand;
import frc.robot.commands.autoCommandBlocks.autoHuntTargetCommand;
import frc.robot.commands.autoCommandBlocks.autoShootNoteCommand;
import frc.robot.commands.autoCommandBlocks.autoSpinUpCommand;
import frc.robot.commands.conveyerCommands.ConveyerIntakeTillThirdSensor;
import frc.robot.commands.intakeCommands.intakeTimeCommand;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.PhotonvisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class note1Block extends SequentialCommandGroup {
  /** Creates a new note1Block. */
  public note1Block(DriveSubsystem drive, PhotonvisionSubsystem photon, IntakeSubsystem intake, ShooterSubsystem shooter, ConveyerSubsystem conveyer, LightSubsystem led) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new autoDigestNoteCommand(intake, shooter, conveyer),
      new ParallelDeadlineGroup(

        new SequentialCommandGroup(
          // new intakeTimeCommand(intake, 0, 0, 500), 
          new ConveyerIntakeTillThirdSensor(conveyer),
          new autoShootNoteCommand(drive, photon, led, conveyer)
        ), 

   
        new autoSpinUpCommand(led, shooter)
      ) 

    );
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommandBlocks;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.conveyerConstants;
import frc.robot.commands.autoCommands.autoTools.autoDriveForwardSetDistance;
import frc.robot.commands.conveyerCommands.ConveyerIntakeTillThirdSensor;
import frc.robot.commands.conveyerCommands.ConveyerTimeCommand;
import frc.robot.commands.conveyerCommands.conveyTillFirstSensor;
import frc.robot.commands.conveyerCommands.conveyerTillSensorTwo;
import frc.robot.commands.intakeCommands.IntakeCommand;
import frc.robot.commands.intakeCommands.intakeTimeCommand;
import frc.robot.commands.visionCommands.visionTurnCommand;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoEatNoteCommand extends SequentialCommandGroup {
  /** Creates a new autoEatNoteCommand. */
  public autoEatNoteCommand(DriveSubsystem drive, ConveyerSubsystem conveyer, IntakeSubsystem intake, VisionSubsystem vision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      new SequentialCommandGroup(
        new visionTurnCommand(drive, vision, 0, false, VisionConstants.fineAlighnmentTolerance), 

        new ParallelDeadlineGroup(
            new conveyerTillSensorTwo(conveyer), 
            new autoDriveForwardSetDistance(drive, DriveConstants.autoDriveForwardAndIntakeDistance, 0.50),
            new IntakeCommand(intake, IntakeConstants.intakeSpeed)
        )
      )

    );
  }
}

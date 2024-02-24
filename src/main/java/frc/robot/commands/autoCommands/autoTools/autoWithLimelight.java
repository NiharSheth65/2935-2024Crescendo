// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands.autoTools;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.intakeCommands.intakeTimeCommand;
import frc.robot.commands.visionCommands.visionDriveCommand;
import frc.robot.commands.visionCommands.visionTurnCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoWithLimelight extends SequentialCommandGroup {
  /** Creates a new autoWithLimelight. */
  public autoWithLimelight(DriveSubsystem drive, VisionSubsystem vision, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new visionTurnCommand(drive, vision, 0, false)
      // .andThen(new visionDriveCommand(drive, vision, false, 0, VisionConstants.desiredApproachDistance))
      // .andThen(new visionTurnCommand(drive, vision, 0, false))
      // .andThen(new intakeTimeCommand(intake, 0, 1000, 1000))
      // .andThen(new autoTurnForTime(drive, 650, 0))
      // .andThen(new visionTurnCommand(drive, vision, 0, false))
      // .andThen(new visionDriveCommand(drive, vision, false, 0, VisionConstants.desiredApproachDistance))
      // .andThen(new visionTurnCommand(drive, vision, 0, false))      
    );
  }
}

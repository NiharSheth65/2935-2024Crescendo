// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommandBlocks;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.conveyerConstants;
import frc.robot.commands.conveyerCommands.ConveyOutTillSensorThree;
import frc.robot.commands.conveyerCommands.ConveyerIntakeTillThirdSensor;
import frc.robot.commands.conveyerCommands.ConveyerTimeCommand;
import frc.robot.commands.intakeCommands.IntakeCommand;
import frc.robot.commands.shooterCommands.shooterVelocityCommand;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoDigestNoteCommand extends SequentialCommandGroup {
  /** Creates a new autoDigestNoteCommand. */
  public autoDigestNoteCommand(IntakeSubsystem intake, ShooterSubsystem shooter, ConveyerSubsystem conveyer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // new SequentialCommandGroup(
        new ParallelRaceGroup(
            new IntakeCommand(intake, IntakeConstants.intakeSpeed), 
            new shooterVelocityCommand(shooter, 0, ShooterConstants.shooterHoldInSpeed), 

            new SequentialCommandGroup(
              new ConveyerIntakeTillThirdSensor(conveyer), 
              new ConveyerTimeCommand(conveyer, conveyerConstants.conveyerInSpeed, 325), 
              new ConveyOutTillSensorThree(conveyer), 
              new ConveyerIntakeTillThirdSensor(conveyer)
            )

        )
    );
  }
}

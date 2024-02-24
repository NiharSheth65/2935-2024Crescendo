// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands.blueAutos.blueTwoGamePieceAutos;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.autoCommands.autoTools.autoDriveForwardSetDistance;
import frc.robot.commands.intakeCommands.intakeTimeCommand;
import frc.robot.commands.shooterCommands.shooterTimeCommand;
import frc.robot.commands.visionCommands.visionDriveCommand;
import frc.robot.commands.visionCommands.visionTurnCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class blueClearSideTwoGPAuto extends SequentialCommandGroup {
  /** Creates a new blueClearSidTwoGPAuto. */
  public blueClearSideTwoGPAuto(DriveSubsystem drive, VisionSubsystem vision, IntakeSubsystem intake, ShooterSubsystem shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // shooot first piece 

      // new ParallelRaceGroup(
      //     new shooterTimeCommand(shooter, ShooterConstants.speakerTopMotorSpeed, ShooterConstants.speakerBottomMotorSpeed, 1500), 
      //     new intakeTimeCommand(intake, 0, 1000, 1000), 
      // ) 
    
      // // identify and get to piece 
      // new visionTurnCommand(drive, vision, 0, false)
      // .andThen(new visionDriveCommand(drive, vision, false, 0, VisionConstants.desiredApproachDistance))
      // .andThen(new visionTurnCommand(drive, vision, 0, false))

      // // intake and drive into piece 
      // // currenlty centred around time for intake, switch to sensor ASAP

      // .andThen(
      //   new ParallelRaceGroup(
      //     new intakeTimeCommand(intake, IntakeConstants.intakeSpeed, 1000, 0), 
      //     new autoDriveForwardSetDistance(drive, DriveConstants.autoDriveForwardAndIntakeDistance, 0.60)
      //   ) 
      // )

      // .andThen(
      //   new ParallelRaceGroup(
      //     new shooterTimeCommand(shooter, ShooterConstants.speakerTopMotorSpeed, ShooterConstants.speakerBottomMotorSpeed, 1500), 
      //     new intakeTimeCommand(intake, 0, 1000, 1000), 
      //     new autoDriveForwardSetDistance(drive, -60, 0.60)
      //   ) 
      // )
    );
  }
}

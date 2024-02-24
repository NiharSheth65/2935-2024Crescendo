// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands.blueAutos.blueTwoGamePieceAutos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.conveyerConstants;
import frc.robot.commands.autoCommands.autoTools.autoDriveForwardSetDistance;
import frc.robot.commands.autoCommands.autoTools.autoTurnForTime;
import frc.robot.commands.conveyerCommands.ConveyerCommand;
import frc.robot.commands.conveyerCommands.ConveyerTimeCommand;
import frc.robot.commands.intakeCommands.IntakeCommand;
import frc.robot.commands.intakeCommands.intakeTimeCommand;
import frc.robot.commands.shooterCommands.shooterTimeCommand;
import frc.robot.commands.visionCommands.visionDriveCommand;
import frc.robot.commands.visionCommands.visionTurnCommand;
import frc.robot.commands.wristCommands.wristSetPosition;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class blueCentreTwoGPAuto extends SequentialCommandGroup {
  /** Creates a new blueCentreTwoGPAuto. */
  public blueCentreTwoGPAuto(DriveSubsystem drive, VisionSubsystem vision, IntakeSubsystem intake, ShooterSubsystem shooter, ConveyerSubsystem conveyer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      new ParallelRaceGroup(
          new shooterTimeCommand(shooter, ShooterConstants.speakerTopMotorSpeed, ShooterConstants.speakerBottomMotorSpeed, 1500), 
          new ConveyerTimeCommand(conveyer, conveyerConstants.conveyerInSpeed, 1500), 
          new intakeTimeCommand(intake, 0, 1000, 1000) 
      ) 
    
  
      .andThen(new visionTurnCommand(drive, vision, 0, false, VisionConstants.roughAlignmentTolerance))
      .andThen(new visionDriveCommand(drive, vision, false, 0, VisionConstants.desiredApproachDistance))
      .andThen(new visionTurnCommand(drive, vision, 0, false, VisionConstants.roughAlignmentTolerance)), 

      new ParallelRaceGroup(    
         new autoDriveForwardSetDistance(drive, 15, 0.5)
          .alongWith(new IntakeCommand(intake, IntakeConstants.intakeSpeed))
      ) 


      .andThen(
        new ParallelCommandGroup(
          new shooterTimeCommand(shooter, ShooterConstants.speakerTopMotorSpeed, ShooterConstants.speakerBottomMotorSpeed, 3000), 
          new intakeTimeCommand(intake, IntakeConstants.intakeSpeed, 1000, 1000), 
          new ConveyerTimeCommand(conveyer, conveyerConstants.conveyerInSpeed, 3000), 
          new autoDriveForwardSetDistance(drive, -40, 0.60)
        ) 
      )

    );
  }
}

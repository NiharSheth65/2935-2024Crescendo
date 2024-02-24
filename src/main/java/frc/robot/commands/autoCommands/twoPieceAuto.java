// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.autoCommands.autoTools.autoDriveForwardSetDistance;
import frc.robot.commands.intakeCommands.IntakeCommand;
import frc.robot.commands.intakeCommands.intakeTimeCommand;
import frc.robot.commands.shooterCommands.shooterTimeCommand;
import frc.robot.commands.wristCommands.wristSetPosition;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class twoPieceAuto extends SequentialCommandGroup {
  /** Creates a new twoPieceAuto. */
  public twoPieceAuto(DriveSubsystem drive, ShooterSubsystem shooter, IntakeSubsystem intake, WristSubsystem wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      new ParallelRaceGroup(
        new shooterTimeCommand(shooter, ShooterConstants.speakerTopMotorSpeed, ShooterConstants.speakerBottomMotorSpeed, 1000), 
        new intakeTimeCommand(intake, -IntakeConstants.outtakeSpeed, 500, 500) 
      ), 
         
      new ParallelRaceGroup(
        new shooterTimeCommand(shooter, ShooterConstants.speakerTopMotorSpeed, ShooterConstants.speakerBottomMotorSpeed, 2000), 
        new autoDriveForwardSetDistance(drive, 35, 0.40), 
        new wristSetPosition(wrist, WristConstants.wristIntakePosition), 
        new intakeTimeCommand(intake, -IntakeConstants.intakeSpeed, 2000, 0)
      ),    

      new ParallelRaceGroup(
          new shooterTimeCommand(shooter, ShooterConstants.speakerTopMotorSpeed, ShooterConstants.speakerBottomMotorSpeed, 1500), 
          new wristSetPosition(wrist, WristConstants.wristfeedShooterPosition), 
          new intakeTimeCommand(intake, 0, 1500, 0),
          new autoDriveForwardSetDistance(drive, -32, 0.65) 
      ), 
    

      new ParallelCommandGroup(
        new shooterTimeCommand(shooter, ShooterConstants.speakerTopMotorSpeed, ShooterConstants.speakerBottomMotorSpeed, 1000), 
        new intakeTimeCommand(intake, -IntakeConstants.outtakeSpeed, 500, 500)
      ) 

      // new autoDelayCommand(500), 
      // new autoDriveForwardSetDistance(drive, -60),
      // new autoDelayCommand(500), 
      // new autoTurnOnHeadingCommand(drive, 40), 
      // new autoDelayCommand(500), 
      // new autoDriveForwardSetDistance(drive, 90)
    );
  }
}

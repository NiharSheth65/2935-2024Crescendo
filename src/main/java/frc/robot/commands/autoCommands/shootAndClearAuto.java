// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.autoCommands.autoTools.autoDriveForwardSetDistance;
import frc.robot.commands.conveyerCommands.ConveyerIntakeTillThirdSensor;
import frc.robot.commands.conveyerCommands.conveyerTillSensorCleared;
import frc.robot.commands.intakeCommands.IntakeCommand;
import frc.robot.commands.ledCommands.truckCommand;
import frc.robot.commands.shooterCommands.shooterHasReachedVelocityCommand;
import frc.robot.commands.shooterCommands.shooterVelocityCommand;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.PhotonvisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TruckLightSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class shootAndClearAuto extends SequentialCommandGroup {
  /** Creates a new shootAndClearAuto. */
  public shootAndClearAuto(DriveSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter, ConveyerSubsystem conveyer,  TruckLightSubsystem truck, LightSubsystem led) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelRaceGroup(
        
        new truckCommand(truck, 1.0), 
        new shooterVelocityCommand(shooter, ShooterConstants.speakerTopMotorSpeed, ShooterConstants.speakerBottomMotorSpeed), 

        new SequentialCommandGroup(

          new shooterHasReachedVelocityCommand(led, shooter, ShooterConstants.speakerTopMotorSpeed, ShooterConstants.speakerBottomMotorSpeed, false), 

          // NOTE 1 ******************************************
          new ParallelRaceGroup(
                        
            new IntakeCommand(intake, IntakeConstants.intakeSpeed),

            new SequentialCommandGroup( 
                    new ConveyerIntakeTillThirdSensor(conveyer), 
                    new conveyerTillSensorCleared(conveyer), 
                    new autoDriveForwardSetDistance(drive, 80, DriveConstants.autoDriveSpeed)
            ) 
          )
        )
                    
      )
    );
  }
}

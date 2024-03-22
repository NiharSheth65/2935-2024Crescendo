// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands.blueAutos.blueMiddelGrabStageSidAuto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.autoCommands.autoTools.autoDriveForwardSetDistance;
import frc.robot.commands.autoCommands.autoTools.autoTurnCommand;
import frc.robot.commands.conveyerCommands.ConveyerIntakeTillThirdSensor;
import frc.robot.commands.conveyerCommands.conveyTillFirstSensor;
import frc.robot.commands.conveyerCommands.conveyerTillSensorCleared;
import frc.robot.commands.intakeCommands.IntakeCommand;
import frc.robot.commands.ledCommands.truckCommand;
import frc.robot.commands.photonvisionCommands.photonVisionDriveAndAlignCommand;
import frc.robot.commands.photonvisionCommands.photonVisionLookForCommand;
import frc.robot.commands.shooterCommands.shooterHasReachedVelocityCommand;
import frc.robot.commands.shooterCommands.shooterVelocityCommand;
import frc.robot.commands.visionCommands.visionDriveCommand;
import frc.robot.commands.visionCommands.visionTurnCommand;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TruckLightSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class blueMiddlePickUp extends SequentialCommandGroup {
  /** Creates a new blueMiddlePickUp. */
  public blueMiddlePickUp(DriveSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter, ConveyerSubsystem conveyer, TruckLightSubsystem truck, LightSubsystem led, VisionSubsystem vision) {
    
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
                    new conveyerTillSensorCleared(conveyer)
            ) 
                    

          ), 


          // note 2 **********************************************************************************
          new ParallelRaceGroup(

            new SequentialCommandGroup(


              // drive out to centre 

              new autoDriveForwardSetDistance(drive, 225, DriveConstants.autoDriveSpeed), 
              new autoTurnCommand(drive, -60, false), 
              new autoDriveForwardSetDistance(drive, 20, DriveConstants.autoDriveSpeed), 
           
              // note 2 pick up 
              new ParallelDeadlineGroup(

                    new SequentialCommandGroup(
                      new visionTurnCommand(drive, vision, 1, false, VisionConstants.roughAlignmentTolerance)
                      .andThen(new visionDriveCommand(drive, vision, false, 1, VisionConstants.desiredApproachDistance))
                      .andThen(new visionTurnCommand(drive, vision, 1, false, VisionConstants.fineAlighnmentTolerance))
                    )
                    // new IntakeCommand(intake, IntakeConstants.intakeSpeed)

                )


              // note 2 pick up continued 

                .andThen(
                  new ParallelDeadlineGroup(
                    new conveyTillFirstSensor(conveyer), 
                    new autoDriveForwardSetDistance(drive, DriveConstants.autoDriveForwardAndIntakeDistance, DriveConstants.autoDriveLimelightSpeed),
                    new IntakeCommand(intake, IntakeConstants.intakeSpeed)
               
                  )
                )

              // note two shoot 

                .andThen(

                  new ParallelRaceGroup(
                      new IntakeCommand(intake, IntakeConstants.intakeSpeed),

                      new SequentialCommandGroup(

                        new ParallelCommandGroup(
                          // new photonVisionDriveAndAlignCommand(photon, drive, 0, 0, false), 
                          new ConveyerIntakeTillThirdSensor(conveyer)
                          // new autoDriveForwardSetDistance(drive, -120, DriveConstants.autoDriveSpeed)
                          
                        )

                        // new autoTurnCommand(drive, 70, false), 

              

                        // new conveyerTillSensorCleared(conveyer)
                      )
                  )
              )
            )
          )

            // // note 3 *******************************************

            // new ParallelRaceGroup(

            //   // run shooter in the background

            //   // every thing else for note 3 
            //   new SequentialCommandGroup(
            //     // turn to find piece 
            //     new autoTurnCommand(drive, -90, false), 
            //     new autoDriveForwardSetDistance(drive, 30, DriveConstants.autoDriveSpeed), 
            //     new autoTurnCommand(drive, -35, false), 

            //     // find piece 
            //     new ParallelDeadlineGroup(
            //           new SequentialCommandGroup(
            //             new visionTurnCommand(drive, vision, 1, false, VisionConstants.roughAlignmentTolerance)
            //             .andThen(new visionDriveCommand(drive, vision, false, 1, VisionConstants.desiredApproachDistance))
            //             .andThen(new visionTurnCommand(drive, vision, 1, false, VisionConstants.fineAlighnmentTolerance))
            //           )
            //     ), 

            //     // drive into, intake and bring peice up 
            //     new ParallelDeadlineGroup(
            //       new conveyTillFirstSensor(conveyer),
            //       new autoDriveForwardSetDistance(drive, DriveConstants.autoDriveForwardAndIntakeDistance/1.5, DriveConstants.autoDriveLimelightSpeed),
            //       new IntakeCommand(intake, IntakeConstants.intakeSpeed)
            //     )       
            //   )
            // )
        )
      )
    );
  }
}

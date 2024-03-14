// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands.blueAutos.blueFourGamePieceAuto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.conveyerConstants;
import frc.robot.Constants.photonVisionConstants;
import frc.robot.commands.autoCommands.autoTools.autoDriveForwardSetDistance;
import frc.robot.commands.autoCommands.autoTools.autoTurnCommand;
import frc.robot.commands.conveyerCommands.ConveyerCommand;
import frc.robot.commands.conveyerCommands.ConveyerIntakeTillThirdSensor;
import frc.robot.commands.conveyerCommands.conveyTillFirstSensor;
import frc.robot.commands.conveyerCommands.conveyerTillSensorCleared;
import frc.robot.commands.intakeCommands.IntakeCommand;
import frc.robot.commands.ledCommands.truckCommand;
import frc.robot.commands.photonvisionCommands.photonVisionDriveAndAlignCommand;
import frc.robot.commands.photonvisionCommands.photonVisionLookForCommand;
import frc.robot.commands.shooterCommands.shooterVelocityCommand;
import frc.robot.commands.visionCommands.visionDriveCommand;
import frc.robot.commands.visionCommands.visionTurnCommand;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonvisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TruckLightSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class blueCentreClearFourGPAuto extends SequentialCommandGroup {
  /** Creates a new blueCentreClearFourGPAuto. */
  public blueCentreClearFourGPAuto(DriveSubsystem drive, VisionSubsystem vision, IntakeSubsystem intake, ShooterSubsystem shooter, ConveyerSubsystem conveyer, PhotonvisionSubsystem photon, TruckLightSubsystem truck) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

     new ParallelRaceGroup(
        
        new truckCommand(truck, 1.0), 
        new shooterVelocityCommand(shooter, ShooterConstants.speakerTopMotorSpeed, ShooterConstants.speakerBottomMotorSpeed), 

        new SequentialCommandGroup(
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
                          new ConveyerIntakeTillThirdSensor(conveyer),
                          new autoDriveForwardSetDistance(drive, -45, DriveConstants.autoDriveSpeed)
                          
                        ), 

              

                        new conveyerTillSensorCleared(conveyer)
                      )
                  )
              )
            )
          ), 

            // note 3 *******************************************

            new ParallelRaceGroup(

              // run shooter in the background

              // every thing else for note 3 
              new SequentialCommandGroup(
                // turn to find piece 
                new autoDriveForwardSetDistance(drive, 10, DriveConstants.autoDriveSpeed), 
                new autoTurnCommand(drive, -35, false), 

                // find piece 
                new ParallelDeadlineGroup(
                      new SequentialCommandGroup(
                        new visionTurnCommand(drive, vision, 1, false, VisionConstants.roughAlignmentTolerance)
                        .andThen(new visionDriveCommand(drive, vision, false, 1, VisionConstants.desiredApproachDistance))
                        .andThen(new visionTurnCommand(drive, vision, 1, false, VisionConstants.fineAlighnmentTolerance))
                      )
                ), 

                // drive into, intake and bring peice up 
                new ParallelDeadlineGroup(
                  new conveyTillFirstSensor(conveyer),
                  new autoDriveForwardSetDistance(drive, DriveConstants.autoDriveForwardAndIntakeDistance/1.5, DriveConstants.autoDriveLimelightSpeed),
                  new IntakeCommand(intake, IntakeConstants.intakeSpeed)
                ), 

                // new autoTurnCommand(drive, 25, false), 
                new ParallelRaceGroup(
                  new autoTurnCommand(drive, 25, false), 
                  new photonVisionLookForCommand(photon, false, photonVisionConstants.speakerMiddleBlueID)
                ), 

                new photonVisionDriveAndAlignCommand(photon, drive, 0, 0, false, photonVisionConstants.speakerMiddleBlueID), 


                // drive up and run conveyer 
                new ParallelDeadlineGroup(

                  new ParallelDeadlineGroup(
                    new autoDriveForwardSetDistance(drive, -40, DriveConstants.autoDriveSpeed), 
                    new ConveyerIntakeTillThirdSensor(conveyer)
                  ),  

                  new IntakeCommand(intake, IntakeConstants.intakeSpeed)
                ), 

                // turn to get april tags in view 
                // new autoTurnCommand(drive, 15, false), 
                // align to tag 
                // new photonVisionDriveAndAlignCommand(photon, drive, 0, 0, false, photonVisionConstants.speakerMiddleBlueID), 

                // final step to shoot
                new ParallelRaceGroup(

                // run intake and shooter
                  new IntakeCommand(intake, IntakeConstants.intakeSpeed),

                  // drive to sub woofer and let of not once close enough 
                  new SequentialCommandGroup(
                    new autoDriveForwardSetDistance(drive, -20, DriveConstants.autoDriveSpeed), 
                    new conveyerTillSensorCleared(conveyer)
                  )
                )

        
              )
            ), 

            // note 4 *******************************************

            new ParallelRaceGroup(

              // run shooter in the background

              // every thing else for note 3 
              new SequentialCommandGroup(
                // turn to find piece 
                new autoDriveForwardSetDistance(drive, 6, DriveConstants.autoDriveSpeed), 
                new autoTurnCommand(drive, 50, false), 
                // new autoDriveForwardSetDistance(drive, 20, DriveConstants.autoDriveSpeed), 
                // find piece 
                new ParallelDeadlineGroup(
                      new SequentialCommandGroup(
                        new visionTurnCommand(drive, vision, 1, false, VisionConstants.roughAlignmentTolerance)
                        .andThen(new visionDriveCommand(drive, vision, false, 1, VisionConstants.desiredApproachDistance))
                        .andThen(new visionTurnCommand(drive, vision, 1, false, VisionConstants.fineAlighnmentTolerance))
                      )
                ), 


                // drive into, intake and bring peice up 
                new ParallelDeadlineGroup(
                  new conveyTillFirstSensor(conveyer), 
                  new autoDriveForwardSetDistance(drive, DriveConstants.autoDriveForwardAndIntakeDistance, DriveConstants.autoDriveLimelightSpeed),
                  new IntakeCommand(intake, IntakeConstants.intakeSpeed)
          
                ), 

                new ParallelDeadlineGroup(
                  new autoDriveForwardSetDistance(drive, -10, DriveConstants.autoDriveSpeed), 
                  new ConveyerIntakeTillThirdSensor(conveyer), 
                  new IntakeCommand(intake, IntakeConstants.intakeSpeed)
                ), 
                

                new ParallelRaceGroup(
                  new autoTurnCommand(drive, -30, false), 
                  new photonVisionLookForCommand(photon, false, photonVisionConstants.speakerMiddleBlueID), 
                  new ConveyerCommand(conveyer, conveyerConstants.conveyerInSpeed/5)
                ), 

                new photonVisionDriveAndAlignCommand(photon, drive, 0, 0, false, photonVisionConstants.speakerMiddleBlueID), 

        
                // final step to shoot
                new ParallelDeadlineGroup(
                  new autoDriveForwardSetDistance(drive, -50, DriveConstants.autoDriveSpeed),
                  new IntakeCommand(intake, IntakeConstants.intakeSpeed), 
                  new ConveyerIntakeTillThirdSensor(conveyer)
     
                ), 

                new conveyerTillSensorCleared(conveyer)

        
              )
            )

        )
      )
    );
  }
}
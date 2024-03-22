// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands.autoNoteBlocks;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
public class autoTwoPieceBlocks extends SequentialCommandGroup {
  /** Creates a new autoTwoPieceBlocks. */
  public autoTwoPieceBlocks(DriveSubsystem drive, VisionSubsystem vision, PhotonvisionSubsystem photon, IntakeSubsystem intake, ShooterSubsystem shooter, ConveyerSubsystem conveyer, LightSubsystem led, TruckLightSubsystem truck) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new note1Block(drive, photon, intake, shooter, conveyer, led), 
      new note2Block(drive, vision, photon, intake, shooter, conveyer, led, truck)
    );
  }
}

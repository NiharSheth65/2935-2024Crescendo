// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands.redAutos.redBlocksAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autoCommands.autoNoteBlocks.note1Block;
import frc.robot.commands.autoCommands.autoNoteBlocks.note2Block;
import frc.robot.commands.autoCommands.autoNoteBlocks.note3Block;
import frc.robot.commands.autoCommands.autoNoteBlocks.note4Block;
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
public class fourPieceBlockAuto extends SequentialCommandGroup {
  /** Creates a new fourPieceBlockAuto. */
  public fourPieceBlockAuto(DriveSubsystem drive, VisionSubsystem vision, IntakeSubsystem intake, ShooterSubsystem shooter, ConveyerSubsystem conveyer, PhotonvisionSubsystem photon, TruckLightSubsystem truck, LightSubsystem led) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new note1Block(drive, photon, intake, shooter, conveyer, led), 
      new note2Block(drive, vision, photon, intake, shooter, conveyer, led, truck), 
      new note3Block(drive, vision, photon, intake, shooter, conveyer, led, truck), 
      new note4Block(drive, vision, photon, intake, shooter, conveyer, led, truck)
    );
  }
}

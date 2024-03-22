// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.conveyerConstants;
import frc.robot.Constants.photonVisionConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.autoCommandBlocks.autoDigestNoteCommand;
import frc.robot.commands.autoCommandBlocks.autoEatNoteCommand;
import frc.robot.commands.autoCommandBlocks.autoHuntNoteCommand;
import frc.robot.commands.autoCommandBlocks.autoHuntTargetCommand;
import frc.robot.commands.autoCommandBlocks.autoShootNoteCommand;
import frc.robot.commands.autoCommandBlocks.autoSpinUpCommand;
import frc.robot.commands.autoCommands.autoTwoPieceCentreNoVision;
import frc.robot.commands.autoCommands.doNothingAuto;
import frc.robot.commands.autoCommands.driveAuto;
import frc.robot.commands.autoCommands.shootAndClearAuto;
import frc.robot.commands.autoCommands.autoNoteBlocks.autoThreePieceBlocks;
import frc.robot.commands.autoCommands.autoNoteBlocks.autoTwoPieceBlocks;
import frc.robot.commands.autoCommands.autoNoteBlocks.note1Block;
import frc.robot.commands.autoCommands.autoNoteBlocks.note2Block;
import frc.robot.commands.autoCommands.autoNoteBlocks.note3Block;
import frc.robot.commands.autoCommands.autoNoteBlocks.note4Block;
import frc.robot.commands.autoCommands.autoTools.autoDriveForwardSetDistance;
import frc.robot.commands.autoCommands.autoTools.autoTurnCommand;
import frc.robot.commands.autoCommands.autoTools.autoTurnForTime;
import frc.robot.commands.autoCommands.autoTools.autoTurnOnHeadingCommand;
import frc.robot.commands.autoCommands.autoTools.autoWithLimelight;
import frc.robot.commands.autoCommands.blueAutos.blueBlocksAuto.blueBlockFourPiece;
import frc.robot.commands.autoCommands.blueAutos.blueFourGamePieceAuto.blueCentreClearFourGPAuto;
import frc.robot.commands.autoCommands.blueAutos.blueMiddelGrabStageSidAuto.blueMiddlePickUp;
import frc.robot.commands.autoCommands.blueAutos.blueThreeGamePieceAutos.blueCentreClearThreeGPAuto;
import frc.robot.commands.autoCommands.blueAutos.blueThreeGamePieceAutos.blueCentreStageThreeGPAuto;
import frc.robot.commands.autoCommands.blueAutos.blueTwoGamePieceAutos.blueCentreClearTwoGPAuto;
import frc.robot.commands.autoCommands.blueAutos.blueTwoGamePieceAutos.blueCentreStageTwoGPAuto;
import frc.robot.commands.autoCommands.blueAutos.blueTwoGamePieceAutos.blueCentreTwoGPAuto;
import frc.robot.commands.autoCommands.redAutos.redBlocksAuto.fourPieceBlockAuto;
import frc.robot.commands.autoCommands.redAutos.redBlocksAuto.redTwoNoteBlock;
import frc.robot.commands.autoCommands.redAutos.redFourGamePieceAutos.redCentreStageFourGPAuto;
import frc.robot.commands.autoCommands.redAutos.redThreeGamePieceAutos.redCentreClearThreeGPAuto;
import frc.robot.commands.autoCommands.redAutos.redThreeGamePieceAutos.redCentreStageThreeGPAuto;
import frc.robot.commands.autoCommands.redAutos.redTwoGamePieceAutos.redCentreClearTwoGPAuto;
import frc.robot.commands.autoCommands.redAutos.redTwoGamePieceAutos.redCentreStageTwoGPAuto;
import frc.robot.commands.autoCommands.redAutos.redTwoGamePieceAutos.redCentreTwoGPAuto;
import frc.robot.commands.climbCommands.climbManualSpeedCommand;
import frc.robot.commands.climbCommands.climbToPositionCommand;
import frc.robot.commands.conveyerCommands.ConveyOutTillSensorThree;
import frc.robot.commands.conveyerCommands.ConveyerCommand;
import frc.robot.commands.conveyerCommands.ConveyerIntakeTillThirdSensor;
import frc.robot.commands.conveyerCommands.ConveyerTimeCommand;
import frc.robot.commands.conveyerCommands.conveyTillFirstSensor;
import frc.robot.commands.drivetrainCommands.DefaultDriveCommand;
import frc.robot.commands.drivetrainCommands.DriveTurnCommand;
import frc.robot.commands.drivetrainCommands.DriveVelocityControl;
import frc.robot.commands.intakeCommands.IntakeCommand;
import frc.robot.commands.intakeCommands.IntakeVelocityCommand;
import frc.robot.commands.intakeCommands.IntakeWithSensorCommand;
import frc.robot.commands.intakeCommands.intakeTillFirstSensor;
import frc.robot.commands.intakeCommands.intakeTimeCommand;
import frc.robot.commands.ledCommands.LedCommand;
import frc.robot.commands.ledCommands.ledRevedUpCommand;
import frc.robot.commands.ledCommands.truckCommand;
import frc.robot.commands.photonvisionCommands.photonVisionAlignToTagCommand;
import frc.robot.commands.photonvisionCommands.photonVisionDriveAndAlignCommand;
import frc.robot.commands.photonvisionCommands.photonVisionDriveCommand;
// import frc.robot.commands.photonvisionCommands.photonVisionDriveAndAlignCommand;
import frc.robot.commands.shooterCommands.shootSetSpeedCommand;
import frc.robot.commands.shooterCommands.shooterTimeCommand;
import frc.robot.commands.shooterCommands.shooterVelocityCommand;
import frc.robot.commands.visionCommands.visionDriveCommand;
import frc.robot.commands.visionCommands.visionReadCommand;
import frc.robot.commands.visionCommands.visionSweepCommand;
import frc.robot.commands.visionCommands.visionTurnCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.PhotonvisionSubsystem;
// import frc.robot.subsystems.PhotonvisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TruckLightSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.Optional;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.ADXL345_I2C.AllAxes;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem(); 
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ConveyerSubsystem m_ConveyerSubsystem = new ConveyerSubsystem();  
  private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem(); 
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem(); 
  private final PhotonvisionSubsystem m_PhotonvisionSubsystem = new PhotonvisionSubsystem(); 
  private final LightSubsystem m_LightSubsystem = new LightSubsystem(); 
  private final TruckLightSubsystem m_TruckLightSubsystem = new TruckLightSubsystem(); 

  private final Joystick joystick = new Joystick(OperatorConstants.primaryControllerPort); 
  private final Joystick joystickSecondary = new Joystick(OperatorConstants.secondaryControllerPort); 

  private final JoystickButton BUTTON_A_PRIMARY = new JoystickButton(joystick, OperatorConstants.BUTTON_A_PORT); 
  private final JoystickButton BUTTON_B_PRIMARY = new JoystickButton(joystick, OperatorConstants.BUTTON_B_PORT); 
  private final JoystickButton BUTTON_Y_PRIMARY = new JoystickButton(joystick, OperatorConstants.BUTTON_Y_PORT); 
  private final JoystickButton BUTTON_X_PRIMARY = new JoystickButton(joystick, OperatorConstants.BUTTON_X_PORT); 
  
  private final JoystickButton BUTTON_RB_PRIMARY = new JoystickButton(joystick, OperatorConstants.BUTTON_RB_PORT); 
  private final JoystickButton BUTTON_LB_PRIMARY = new JoystickButton(joystick, OperatorConstants.BUTTON_LB_PORT); 

  // secondary controller 
  private final JoystickButton BUTTON_A_SECONDARY = new JoystickButton(joystickSecondary, OperatorConstants.BUTTON_A_PORT); 
  private final JoystickButton BUTTON_B_SECONDARY = new JoystickButton(joystickSecondary, OperatorConstants.BUTTON_B_PORT); 
  private final JoystickButton BUTTON_Y_SECONDARY = new JoystickButton(joystickSecondary, OperatorConstants.BUTTON_Y_PORT); 
  private final JoystickButton BUTTON_X_SECONDARY = new JoystickButton(joystickSecondary, OperatorConstants.BUTTON_X_PORT); 
  
  private final JoystickButton BUTTON_RB_SECONDARY = new JoystickButton(joystickSecondary, OperatorConstants.BUTTON_RB_PORT); 
  private final JoystickButton BUTTON_LB_SECONDARY = new JoystickButton(joystickSecondary, OperatorConstants.BUTTON_LB_PORT); 

  private final CommandGenericHID controllerPrimary = new CommandGenericHID(OperatorConstants.primaryControllerPort);  
  private final CommandGenericHID controllerSecondary = new CommandGenericHID(OperatorConstants.secondaryControllerPort);  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);


  private final Command m_blueCentreClearSideFourPieceAuto = new blueCentreClearFourGPAuto(m_DriveSubsystem, m_VisionSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_PhotonvisionSubsystem, m_TruckLightSubsystem, m_LightSubsystem); 
  
  private final Command m_blueCentreClearSideThreePieceAuto = new blueCentreClearThreeGPAuto(m_DriveSubsystem, m_VisionSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_PhotonvisionSubsystem, m_TruckLightSubsystem, m_LightSubsystem); 
  private final Command m_blueCentreStageSideThreePieceAuto = new blueCentreStageThreeGPAuto(m_DriveSubsystem, m_VisionSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_PhotonvisionSubsystem, m_TruckLightSubsystem, m_LightSubsystem); 
  
  private final Command m_blueCentreCentreSideTwoPieceAuto = new blueCentreTwoGPAuto(m_DriveSubsystem, m_VisionSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_PhotonvisionSubsystem, m_TruckLightSubsystem, m_LightSubsystem); 
  private final Command m_blueCentreStageSideTwoPieceAuto = new blueCentreStageTwoGPAuto(m_DriveSubsystem, m_VisionSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_PhotonvisionSubsystem, m_TruckLightSubsystem, m_LightSubsystem); 
  private final Command m_blueCentreClearSideTwoPieceAuto = new blueCentreClearTwoGPAuto(m_DriveSubsystem, m_VisionSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_PhotonvisionSubsystem, m_TruckLightSubsystem, m_LightSubsystem); 

  private final Command m_redCentreStageSideFourPieceAuto = new redCentreStageFourGPAuto(m_DriveSubsystem, m_VisionSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_PhotonvisionSubsystem, m_TruckLightSubsystem, m_LightSubsystem); 
  
  private final Command m_redCentreClearSideThreePieceAuto = new redCentreClearThreeGPAuto(m_DriveSubsystem, m_VisionSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_PhotonvisionSubsystem, m_TruckLightSubsystem, m_LightSubsystem); 
  private final Command m_redCentreStageSideThreePieceAuto = new redCentreStageThreeGPAuto(m_DriveSubsystem, m_VisionSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_PhotonvisionSubsystem, m_TruckLightSubsystem, m_LightSubsystem); 
  
  private final Command m_redCentreCentreSideTwoPieceAuto = new redCentreTwoGPAuto(m_DriveSubsystem, m_VisionSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_PhotonvisionSubsystem, m_TruckLightSubsystem, m_LightSubsystem); 
  private final Command m_redCentreStageSideTwoPieceAuto = new redCentreStageTwoGPAuto(m_DriveSubsystem, m_VisionSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_PhotonvisionSubsystem, m_TruckLightSubsystem, m_LightSubsystem); 
  private final Command m_redCentreClearSideTwoPieceAuto = new redCentreClearTwoGPAuto(m_DriveSubsystem, m_VisionSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_PhotonvisionSubsystem, m_TruckLightSubsystem, m_LightSubsystem); 

  private final Command m_clearAuto = new driveAuto(m_DriveSubsystem); 
  private final Command m_shootAndClearAuto = new shootAndClearAuto(m_DriveSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_TruckLightSubsystem, m_LightSubsystem); 
  private final Command m_doNothingAuto = new doNothingAuto(); 
  private final Command m_twoPieceNoVision = new autoTwoPieceCentreNoVision(m_DriveSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_TruckLightSubsystem, m_LightSubsystem); 

  private final Command m_blueMiddlePickUp = new blueMiddlePickUp(m_DriveSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_TruckLightSubsystem, m_LightSubsystem, m_VisionSubsystem);

  private final Command m_note1Command = new note1Block(m_DriveSubsystem, m_PhotonvisionSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_LightSubsystem); 
  private final Command m_note2Command = new note2Block(m_DriveSubsystem, m_VisionSubsystem, m_PhotonvisionSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_LightSubsystem, m_TruckLightSubsystem);
  private final Command m_note3Command = new note3Block(m_DriveSubsystem, m_VisionSubsystem, m_PhotonvisionSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_LightSubsystem, m_TruckLightSubsystem);
  private final Command m_note4Command = new note4Block(m_DriveSubsystem, m_VisionSubsystem, m_PhotonvisionSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_LightSubsystem, m_TruckLightSubsystem);

  private final Command m_redFourPieceBlocks = new fourPieceBlockAuto(m_DriveSubsystem, m_VisionSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_PhotonvisionSubsystem, m_TruckLightSubsystem, m_LightSubsystem);
  private final Command m_blueFourPieceBlocks = new blueBlockFourPiece(m_DriveSubsystem, m_VisionSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_PhotonvisionSubsystem, m_TruckLightSubsystem, m_LightSubsystem); 

  private final Command m_blocksTwoPiece = new autoTwoPieceBlocks(m_DriveSubsystem, m_VisionSubsystem, m_PhotonvisionSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_LightSubsystem, m_TruckLightSubsystem);
  private final Command m_blocksThreePiece = new autoThreePieceBlocks(m_DriveSubsystem, m_VisionSubsystem, m_PhotonvisionSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_LightSubsystem, m_TruckLightSubsystem);
  
  // private final Command m_note1 = new note1Block(m_DriveSubsystem, m_PhotonvisionSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_LightSubsystem);
  SendableChooser<Command> m_autoChooser = new SendableChooser<>(); 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    // CameraServer.startAutomaticCapture(); 

    // blue side autos 

    m_autoChooser.setDefaultOption("BLUE - FOUR PIECE CLEAR SIDE", m_blueCentreClearSideFourPieceAuto);
    m_autoChooser.addOption("NOTE 1", m_note1Command);
    m_autoChooser.addOption("NOTE 2", m_note2Command);
    m_autoChooser.addOption("NOTE 3", m_note3Command);
    m_autoChooser.addOption("NOTE 4", m_note4Command);
    m_autoChooser.addOption("2 piece block", m_blocksTwoPiece);
    m_autoChooser.addOption("3 piece block", m_blocksThreePiece);


    m_autoChooser.addOption("red - 4 piece block", m_redFourPieceBlocks); 
    m_autoChooser.addOption("blue - 4 piece block", m_blueFourPieceBlocks); 
    m_autoChooser.addOption("BLUE TWO NOTE", m_blocksTwoPiece);

    m_autoChooser.addOption("RED - FOUR PIECE STAGE SIDE", m_redCentreStageSideFourPieceAuto);
    
    m_autoChooser.addOption("BLUE - THREE PIECE CENTRE CLEAR SIDE", m_blueCentreClearSideThreePieceAuto);
    m_autoChooser.addOption("BLUE - THREE PIECE CENTRE STAGE SIDE", m_blueCentreStageSideThreePieceAuto);

    m_autoChooser.addOption("RED - THREE PIECE CENTRE CLEAR SIDE", m_redCentreClearSideThreePieceAuto);
    m_autoChooser.addOption("RED - THREE PIECE CENTRE STAGE SIDE", m_redCentreStageSideThreePieceAuto);

    m_autoChooser.addOption("BLUE - TWO PIECE CENTRE CENTRE SIDE", m_blueCentreCentreSideTwoPieceAuto);
    m_autoChooser.addOption("BLUE - TWO PIECE CENTRE CLEAR SIDE", m_blueCentreClearSideFourPieceAuto);
    m_autoChooser.addOption("BLUE - TWO PIECE CENTRE STAGE SIDE", m_blueCentreStageSideTwoPieceAuto);

    m_autoChooser.addOption("RED - TWO PIECE CENTRE CENTRE SIDE", m_redCentreCentreSideTwoPieceAuto);
    m_autoChooser.addOption("RED - TWO PIECE CENTRE CLEAR SIDE", m_redCentreClearSideTwoPieceAuto);
    m_autoChooser.addOption("RED - TWO PIECE CENTRE STAGE SIDE", m_redCentreStageSideTwoPieceAuto);

    m_autoChooser.addOption("DRIVE OUT", m_clearAuto);
    m_autoChooser.addOption("SHOOT AND DRIVE OUT", m_shootAndClearAuto);
    m_autoChooser.addOption("DO NOTHING AUTO", m_doNothingAuto);
    m_autoChooser.addOption("TWO PIECE CENTRE NO VISION", m_twoPieceNoVision);

    m_autoChooser.addOption("BLUE CENTRE GRAB", m_blueMiddlePickUp);


    Shuffleboard.getTab("Autonomous").add(m_autoChooser); 
    
    configureBindings();
    defaultCommands(); 
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void defaultCommands(){
    m_DriveSubsystem.setDefaultCommand(new DefaultDriveCommand(m_DriveSubsystem, joystick));
    m_VisionSubsystem.setDefaultCommand(new visionReadCommand(m_VisionSubsystem));
    m_LightSubsystem.setDefaultCommand(new LedCommand(m_LightSubsystem, m_IntakeSubsystem, m_ConveyerSubsystem));
    m_TruckLightSubsystem.setDefaultCommand(new truckCommand(m_TruckLightSubsystem, 0.0));
  }
  
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`


    // DRIVE CONTROLLER 
    
    // TURNS TRUCK LIGHTS ON AND OFF 

    BUTTON_A_PRIMARY.onTrue(
      new climbToPositionCommand(m_ClimbSubsystem, ClimbConstants.climbFullyExtendedPosition)
    ); 

    BUTTON_A_PRIMARY.onFalse(
      new climbToPositionCommand(m_ClimbSubsystem, ClimbConstants.climbFullyRetractedPosition)
    ); 

    BUTTON_B_PRIMARY.onTrue(
      new autoHuntTargetCommand(m_DriveSubsystem, m_PhotonvisionSubsystem)
    ); 

    BUTTON_B_PRIMARY.onFalse(
      new ParallelRaceGroup(
              new truckCommand(m_TruckLightSubsystem, 0)
              // new autoHuntNoteCommand(m_DriveSubsystem, m_VisionSubsystem)
      )

    ); 

    // AUTO INTAKE 

    BUTTON_Y_PRIMARY.onTrue(
      new SequentialCommandGroup(
        new autoHuntNoteCommand(m_DriveSubsystem, m_VisionSubsystem, m_TruckLightSubsystem), 
        new autoEatNoteCommand(m_DriveSubsystem, m_ConveyerSubsystem, m_IntakeSubsystem, m_VisionSubsystem)
      )
    ); 


    BUTTON_Y_PRIMARY.onFalse(
      new truckCommand(m_TruckLightSubsystem, 0)
    ); 
    
    // SWEEP FOR PIECE 

    BUTTON_X_PRIMARY.onTrue(
      new ParallelCommandGroup(
        new visionSweepCommand(m_VisionSubsystem, 1, false, VisionConstants.roughAlignmentTolerance), 
        new truckCommand(m_TruckLightSubsystem, 1.0)
      )
    ); 
    

    BUTTON_X_PRIMARY.onFalse(
      new visionSweepCommand(m_VisionSubsystem, 1, true, VisionConstants.roughAlignmentTolerance)
    ); 

    controllerPrimary.axisGreaterThan(OperatorConstants.rightTriggerAxis, OperatorConstants.triggerThreshold).toggleOnFalse(new climbManualSpeedCommand(m_ClimbSubsystem, 0)); 
    controllerPrimary.axisGreaterThan(OperatorConstants.rightTriggerAxis, OperatorConstants.triggerThreshold).toggleOnTrue(new climbManualSpeedCommand(m_ClimbSubsystem, ClimbConstants.climbManualExtendSpeed));
    
    controllerPrimary.axisGreaterThan(OperatorConstants.leftTriggerAxis, OperatorConstants.triggerThreshold).toggleOnFalse(new climbManualSpeedCommand(m_ClimbSubsystem, 0)); 
    controllerPrimary.axisGreaterThan(OperatorConstants.leftTriggerAxis, OperatorConstants.triggerThreshold).toggleOnTrue(new climbManualSpeedCommand(m_ClimbSubsystem, ClimbConstants.climbManualRetractSpeed)); 

    // controllerPrimary.axisGreaterThan(OperatorConstants.rightTriggerAxis, OperatorConstants.triggerThreshold).toggleOnFalse(new climbManualSpeedCommand(m_ClimbSubsystem, 0)); 
    // controllerPrimary.axisGreaterThan(OperatorConstants.rightTriggerAxis, OperatorConstants.triggerThreshold).toggleOnTrue(
    //   new climbToPositionCommand(m_ClimbSubsystem, 476));
    
    // controllerPrimary.axisGreaterThan(OperatorConstants.leftTriggerAxis, OperatorConstants.triggerThreshold).toggleOnFalse(new climbManualSpeedCommand(m_ClimbSubsystem, 0)); 
    // controllerPrimary.axisGreaterThan(OperatorConstants.leftTriggerAxis, OperatorConstants.triggerThreshold).toggleOnTrue(
    //   new climbManualSpeedCommand(m_ClimbSubsystem, ClimbConstants.climbManualRetractSpeed)
    //   ); 


    // -----------------------------------------------------------------------------------------------------

    // OPERATOR CONTROLLER

    // USE RIGHT TRIGGER TO SHOOT SPEAKER SPEED

    controllerSecondary.axisGreaterThan(OperatorConstants.rightTriggerAxis, OperatorConstants.triggerThreshold).toggleOnFalse(new shootSetSpeedCommand(m_ShooterSubsystem, 0, 0)); 
    controllerSecondary.axisGreaterThan(OperatorConstants.rightTriggerAxis, OperatorConstants.triggerThreshold).toggleOnTrue(
        new ParallelCommandGroup(
          new shooterVelocityCommand(m_ShooterSubsystem, ShooterConstants.speakerTopMotorSpeed, ShooterConstants.speakerBottomMotorSpeed), 
          new ledRevedUpCommand(m_LightSubsystem, m_ShooterSubsystem, ShooterConstants.speakerTopMotorSpeed, ShooterConstants.speakerBottomMotorSpeed, false)
        )
    ); 

    // USE LEFT TRIGGER TO SHOOT AMP SPEED

    controllerSecondary.axisGreaterThan(OperatorConstants.leftTriggerAxis, OperatorConstants.triggerThreshold).toggleOnFalse(new shootSetSpeedCommand(m_ShooterSubsystem, 0, 0)); 
    controllerSecondary.axisGreaterThan(OperatorConstants.leftTriggerAxis, OperatorConstants.triggerThreshold).toggleOnTrue(
        new ParallelCommandGroup(
          new shooterVelocityCommand(m_ShooterSubsystem, ShooterConstants.ampTopMotorSpeed, ShooterConstants.ampBottomMotorSpeed), 
          new ledRevedUpCommand(m_LightSubsystem, m_ShooterSubsystem, ShooterConstants.ampTopMotorSpeed, ShooterConstants.ampBottomMotorSpeed, false)
        )
    ); 
    

    // RUNS SHOOTER WHEELS INVERTED TO INTAKE FROM SOURCE 
    BUTTON_A_SECONDARY.onTrue(
      new shooterVelocityCommand(m_ShooterSubsystem, -ShooterConstants.ampTopMotorSpeed, -ShooterConstants.ampBottomMotorSpeed)
    ); 

    BUTTON_A_SECONDARY.onFalse(
      new shooterVelocityCommand(m_ShooterSubsystem, 0, 0)
    ); 

    // RUN SHOOTER AMP SPEED AND CENTRE NOTE 

    BUTTON_B_SECONDARY.onTrue(
      
      new SequentialCommandGroup(

        new ParallelRaceGroup(
          new IntakeCommand(m_IntakeSubsystem, IntakeConstants.intakeSpeed), 
          new shooterVelocityCommand(m_ShooterSubsystem, 0, ShooterConstants.shooterHoldInSpeed), 

          new SequentialCommandGroup(
            new ConveyerIntakeTillThirdSensor(m_ConveyerSubsystem), 
            new ConveyerTimeCommand(m_ConveyerSubsystem, conveyerConstants.conveyerInSpeed, 650), 
            new ConveyOutTillSensorThree(m_ConveyerSubsystem)
          )

        ), 

        new ParallelCommandGroup(
          new shooterVelocityCommand(m_ShooterSubsystem, ShooterConstants.ampTopMotorSpeed, ShooterConstants.ampBottomMotorSpeed), 
          new ledRevedUpCommand(m_LightSubsystem, m_ShooterSubsystem, ShooterConstants.ampTopMotorSpeed, ShooterConstants.ampBottomMotorSpeed, false)
        )
    
      )

    ); 

    BUTTON_B_SECONDARY.onFalse(
      new IntakeCommand(m_IntakeSubsystem, 0)
    ); 

    // RUN SHOOTER SPEAKER SPEED AND CENTRE NOTE 

    BUTTON_Y_SECONDARY.onTrue(
      
      new SequentialCommandGroup(

        new ParallelRaceGroup(
          new IntakeCommand(m_IntakeSubsystem, IntakeConstants.intakeSpeed), 
          new shooterVelocityCommand(m_ShooterSubsystem, 0, ShooterConstants.shooterHoldInSpeed), 

          new SequentialCommandGroup(
            new ConveyerIntakeTillThirdSensor(m_ConveyerSubsystem), 
            new ConveyerTimeCommand(m_ConveyerSubsystem, conveyerConstants.conveyerInSpeed, 650), 
            new ConveyOutTillSensorThree(m_ConveyerSubsystem)
          )
          
        ), 

        new ParallelCommandGroup(
          new shooterVelocityCommand(m_ShooterSubsystem, ShooterConstants.speakerTopMotorSpeed, ShooterConstants.speakerBottomMotorSpeed), 
          new ledRevedUpCommand(m_LightSubsystem, m_ShooterSubsystem, ShooterConstants.speakerTopMotorSpeed, ShooterConstants.speakerBottomMotorSpeed, false)
        )
      )

    ); 

    BUTTON_Y_SECONDARY.onFalse(
      new IntakeCommand(m_IntakeSubsystem, 0)
    ); 

    // RUN SHOOTER DISTANCE SPEED AND CENTRE NOTE 

    BUTTON_X_SECONDARY.onTrue(

      new SequentialCommandGroup(

        new ParallelRaceGroup(
          new IntakeCommand(m_IntakeSubsystem, IntakeConstants.intakeSpeed), 
          new shooterVelocityCommand(m_ShooterSubsystem, 0, ShooterConstants.shooterHoldInSpeed), 

          new SequentialCommandGroup(
            new ConveyerIntakeTillThirdSensor(m_ConveyerSubsystem), 
            new ConveyerTimeCommand(m_ConveyerSubsystem, conveyerConstants.conveyerInSpeed, 650), 
            new ConveyOutTillSensorThree(m_ConveyerSubsystem)
          )

        ), 

        new ParallelCommandGroup(
          new shooterVelocityCommand(m_ShooterSubsystem, ShooterConstants.speakerAutoTopMotorSpeed, ShooterConstants.speakerAutoBottomMotorSpeed), 
          new ledRevedUpCommand(m_LightSubsystem, m_ShooterSubsystem, ShooterConstants.speakerAutoTopMotorSpeed, ShooterConstants.speakerAutoBottomMotorSpeed, false)
        )
    
      )
    ); 

    BUTTON_X_SECONDARY.onFalse(
      new shootSetSpeedCommand(m_ShooterSubsystem, 0, 0)
    ); 

    // RUN INTAKE AND CONVEYER MANUAL - SPIT NOTE OUT 

    BUTTON_LB_SECONDARY.onTrue(
      new ParallelCommandGroup(
        new IntakeCommand(m_IntakeSubsystem, IntakeConstants.outtakeSpeed), 
        new ConveyerCommand(m_ConveyerSubsystem, conveyerConstants.conveyerOutSpeed)
      )
    ); 

    BUTTON_LB_SECONDARY.onFalse(
      new ParallelCommandGroup(
        new IntakeCommand(m_IntakeSubsystem, 0), 
        new ConveyerCommand(m_ConveyerSubsystem, 0)
      )
    );   

    // RUN INTAKE AND CONVEYER MANUAL - SUCK NOTE IN
    BUTTON_RB_SECONDARY.onTrue(
      new ParallelCommandGroup(
        new IntakeCommand(m_IntakeSubsystem, IntakeConstants.intakeSpeed), 
        new ConveyerCommand(m_ConveyerSubsystem, conveyerConstants.conveyerInSpeed)
      )
    ); 

    BUTTON_RB_SECONDARY.onFalse(
      new ParallelCommandGroup(
        new IntakeCommand(m_IntakeSubsystem, 0), 
        new ConveyerCommand(m_ConveyerSubsystem, 0)
      )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */


  public static int alliance(){
      Optional<Alliance> ally = DriverStation.getAlliance();
      int alliance = -1; 

      if (ally.isPresent()) {
          if (ally.get() == Alliance.Red) {
              alliance = 0; 
            }
            if (ally.get() == Alliance.Blue) {
              alliance = 1; 
            }
        }
      else {
          alliance = -1; 
      }

      return alliance;
  }




  public void resetEncoders(){
    m_DriveSubsystem.resetEncoders();
    m_ShooterSubsystem.resetEncoders(); 
    m_ClimbSubsystem.resetEncoders();
  }

  public void truckLightOn(){
    m_TruckLightSubsystem.setTruckBrightness(1.0);
  }

  public void truckLightOfF(){
    m_TruckLightSubsystem.setTruckBrightness(0);
  }

  public void autonomousInit(){
    m_DriveSubsystem.resetEncoders();
    m_DriveSubsystem.zeroHeading();
    m_TruckLightSubsystem.setTruckBrightness(1.0);
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    autonomousInit();
    return m_autoChooser.getSelected(); 
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.conveyerConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.autoCommands.twoPieceAuto;
import frc.robot.commands.autoCommands.autoTools.autoDriveForwardSetDistance;
import frc.robot.commands.autoCommands.autoTools.autoTurnForTime;
import frc.robot.commands.autoCommands.autoTools.autoTurnOnHeadingCommand;
import frc.robot.commands.autoCommands.autoTools.autoWithLimelight;
// import frc.robot.commands.autoCommands.blueAutos.blueThreeGamePieceAutos.blueClearSideExitWingThreeGPAuto;
import frc.robot.commands.autoCommands.blueAutos.blueThreeGamePieceAutos.blueClearSideThreeGPAuto;
import frc.robot.commands.autoCommands.blueAutos.blueThreeGamePieceAutos.blueStageSideThreeGpAuto;
import frc.robot.commands.autoCommands.blueAutos.blueTwoGamePieceAutos.blueCentreTwoGPAuto;
import frc.robot.commands.autoCommands.blueAutos.blueTwoGamePieceAutos.blueClearSideTwoGPAuto;
import frc.robot.commands.autoCommands.blueAutos.blueTwoGamePieceAutos.blueStageSideTwoGPAuto;
import frc.robot.commands.conveyerCommands.ConveyerCommand;
import frc.robot.commands.conveyerCommands.ConveyerTimeCommand;
import frc.robot.commands.drivetrainCommands.DefaultDriveCommand;
import frc.robot.commands.drivetrainCommands.DriveVelocityControl;
import frc.robot.commands.intakeCommands.IntakeCommand;
import frc.robot.commands.intakeCommands.IntakeVelocityCommand;
import frc.robot.commands.intakeCommands.intakeTimeCommand;
import frc.robot.commands.shooterCommands.shootSetSpeedCommand;
import frc.robot.commands.shooterCommands.shooterTimeCommand;
import frc.robot.commands.shooterCommands.shooterVelocityCommand;
import frc.robot.commands.visionCommands.visionDriveCommand;
import frc.robot.commands.visionCommands.visionReadCommand;
import frc.robot.commands.visionCommands.visionTurnCommand;
import frc.robot.commands.wristCommands.wristSetPosition;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WristSubsystem;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Joystick;
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
  private final WristSubsystem m_WristSubsystem = new WristSubsystem(); 
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem(); 

  private final Joystick joystick = new Joystick(OperatorConstants.primaryControllerPort); 
  private final Joystick joystickSecondary = new Joystick(OperatorConstants.secondaryControllerPort); 

  private final JoystickButton BUTTON_A_PRIMARY = new JoystickButton(joystick, OperatorConstants.BUTTON_A_PORT); 
  private final JoystickButton BUTTON_B_PRIMARY = new JoystickButton(joystick, OperatorConstants.BUTTON_B_PORT); 
  private final JoystickButton BUTTON_Y_PRIMARY = new JoystickButton(joystick, OperatorConstants.BUTTON_Y_PORT); 
  
  private final JoystickButton BUTTON_RB_PRIMARY = new JoystickButton(joystick, OperatorConstants.BUTTON_RB_PORT); 
  private final JoystickButton BUTTON_LB_PRIMARY = new JoystickButton(joystick, OperatorConstants.BUTTON_LB_PORT); 


  // secondary controller 
  private final JoystickButton BUTTON_A_SECONDARY = new JoystickButton(joystickSecondary, OperatorConstants.BUTTON_A_PORT); 
  private final JoystickButton BUTTON_B_SECONDARY = new JoystickButton(joystickSecondary, OperatorConstants.BUTTON_B_PORT); 
  private final JoystickButton BUTTON_Y_SECONDARY = new JoystickButton(joystickSecondary, OperatorConstants.BUTTON_Y_PORT); 
  
  private final JoystickButton BUTTON_RB_SECONDARY = new JoystickButton(joystickSecondary, OperatorConstants.BUTTON_RB_PORT); 
  private final JoystickButton BUTTON_LB_SECONDARY = new JoystickButton(joystickSecondary, OperatorConstants.BUTTON_LB_PORT); 

  private final CommandGenericHID controllerPrimary = new CommandGenericHID(OperatorConstants.primaryControllerPort);  
  private final CommandGenericHID controllerSecondary = new CommandGenericHID(OperatorConstants.secondaryControllerPort);  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);


  // private final Command m_twoPieceAuto = new twoPieceAuto(m_DriveSubsystem, m_ShooterSubsystem, m_IntakeSubsystem, m_WristSubsystem); 
  // private final Command m_twoPieceVision = new autoWithLimelight(m_DriveSubsystem, m_VisionSubsystem, m_IntakeSubsystem); 

  private final Command m_blueCentreTwoPieceAuto = new blueCentreTwoGPAuto(m_DriveSubsystem, m_VisionSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem); 
  private final Command m_blueStageSideTwoPieceAuto = new blueStageSideTwoGPAuto(m_DriveSubsystem, m_VisionSubsystem, m_IntakeSubsystem, m_ShooterSubsystem); 
  private final Command m_blueClearSideTwoPieceAuto = new blueClearSideTwoGPAuto(m_DriveSubsystem, m_VisionSubsystem, m_IntakeSubsystem, m_ShooterSubsystem); 
  private final Command m_blueStageSideThreePieceAuto = new blueStageSideThreeGpAuto(m_DriveSubsystem, m_VisionSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem); 
  private final Command m_blueClearSideThreePieceAuto = new blueClearSideThreeGPAuto(m_DriveSubsystem, m_VisionSubsystem, m_IntakeSubsystem, m_ShooterSubsystem); 
  // private final Command m_blueClearSideAndExitThreePieceAuto = new blueClearSideExitWingThreeGPAuto(m_DriveSubsystem, m_VisionSubsystem, m_IntakeSubsystem, m_ShooterSubsystem); 

  SendableChooser<Command> m_autoChooser = new SendableChooser<>(); 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    CameraServer.startAutomaticCapture(); 

    m_autoChooser.setDefaultOption("BLUE - TWO PIECE CENTRE", m_blueCentreTwoPieceAuto);
    m_autoChooser.addOption("BLUE - TWO PIECE STAGE", m_blueStageSideTwoPieceAuto);
    m_autoChooser.addOption("BLUE - TWO PIECE CLEAR", m_blueClearSideTwoPieceAuto);
    m_autoChooser.addOption("BLUE - THREE PIECE STAGE", m_blueStageSideThreePieceAuto);
    m_autoChooser.addOption("BLUE - THREE PIECE CLEAR", m_blueClearSideThreePieceAuto);
    // m_autoChooser.addOption("BLUE - THREE PIECE CLEAR SIDE AND EXIT", m_blueClearSideAndExitThreePieceAuto);

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
    // m_WristSubsystem.setDefaultCommand(new wristSetPosition(m_WristSubsystem, 0));
  
  }
  
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`


    BUTTON_B_PRIMARY.toggleOnTrue(
      new autoTurnForTime(m_DriveSubsystem, 500, 0)
    ); 



    BUTTON_Y_PRIMARY.onTrue(
     

      new ParallelDeadlineGroup(
          new SequentialCommandGroup(
            new visionTurnCommand(m_DriveSubsystem, m_VisionSubsystem, 0, false, VisionConstants.roughAlignmentTolerance)
            .andThen(new visionDriveCommand(m_DriveSubsystem, m_VisionSubsystem, false, 0, VisionConstants.desiredApproachDistance))
            .andThen(new visionTurnCommand(m_DriveSubsystem, m_VisionSubsystem, 0, false, VisionConstants.fineAlighnmentTolerance))
          ), 

          new IntakeCommand(m_IntakeSubsystem, IntakeConstants.intakeSpeed)

      )


      .andThen(
        new ParallelDeadlineGroup(
          new IntakeCommand(m_IntakeSubsystem, IntakeConstants.intakeSpeed),
          new autoDriveForwardSetDistance(m_DriveSubsystem, 15, 0.5)
        )
      )

      .andThen(
          new ParallelCommandGroup(
            new intakeTimeCommand(m_IntakeSubsystem, IntakeConstants.intakeSpeed, 400, 0)
            .alongWith(new ConveyerTimeCommand(m_ConveyerSubsystem, conveyerConstants.conveyerInSpeed, 400))
          )
      )
    ); 


      // new limelightReadCommand(m_LimlightSubsystem, false) 
      

    BUTTON_Y_PRIMARY.onFalse(
      new visionTurnCommand(m_DriveSubsystem, m_VisionSubsystem, 0, true, VisionConstants.roughAlignmentTolerance)
      .andThen(new visionDriveCommand(m_DriveSubsystem, m_VisionSubsystem, true, 0,  27))
    ); 

    
    controllerSecondary.axisGreaterThan(OperatorConstants.rightTriggerAxis, OperatorConstants.triggerThreshold).toggleOnFalse(new shootSetSpeedCommand(m_ShooterSubsystem, 0, 0)); 
    controllerSecondary.axisGreaterThan(OperatorConstants.rightTriggerAxis, OperatorConstants.triggerThreshold).toggleOnTrue(new shooterVelocityCommand(m_ShooterSubsystem, ShooterConstants.speakerTopMotorSpeed, ShooterConstants.speakerTopMotorSpeed)); 

    controllerSecondary.axisGreaterThan(OperatorConstants.leftTriggerAxis, OperatorConstants.triggerThreshold).toggleOnFalse(new shootSetSpeedCommand(m_ShooterSubsystem, 0, 0)); 
    controllerSecondary.axisGreaterThan(OperatorConstants.leftTriggerAxis, OperatorConstants.triggerThreshold).toggleOnTrue(new shooterVelocityCommand(m_ShooterSubsystem, ShooterConstants.ampTopMotorSpeed, ShooterConstants.ampBottomMotorSpeed)); 
    
    BUTTON_Y_SECONDARY.onTrue(
      new ParallelCommandGroup(
        new wristSetPosition(m_WristSubsystem, WristConstants.wristIntakePosition)
        // new IntakeCommand(m_IntakeSubsystem,IntakeConstants.outtakeSpeed * 0.7)
      )   
    ); 

    BUTTON_Y_SECONDARY.onFalse(
      new ParallelCommandGroup(
        new wristSetPosition(m_WristSubsystem, WristConstants.wristfeedShooterPosition)
        // new IntakeCommand(m_IntakeSubsystem, 0)
      )
    ); 
    

    BUTTON_RB_SECONDARY.onTrue(
      // new ParallelCommandGroup(
      //        new IntakeCommand(m_IntakeSubsystem, IntakeConstants.intakeSpeed)
      //       //  .alongWith(new ConveyerCommand(m_ConveyerSubsystem, conveyerConstants.conveyerInSpeed))
      // )

      new IntakeCommand(m_IntakeSubsystem, IntakeConstants.intakeSpeed)
      .andThen( 
        new ParallelRaceGroup(
              new IntakeCommand(m_IntakeSubsystem, IntakeConstants.intakeSpeed)
               .alongWith(new ConveyerTimeCommand(m_ConveyerSubsystem, conveyerConstants.conveyerInSpeed, 500))
        )
      )
    ); 



    BUTTON_RB_SECONDARY.onFalse(
       new ParallelCommandGroup(
             new IntakeCommand(m_IntakeSubsystem, 0)
             .alongWith(new ConveyerCommand(m_ConveyerSubsystem, 0))
       )
    );

    BUTTON_LB_SECONDARY.onTrue(


       new ParallelCommandGroup(
          new IntakeCommand(m_IntakeSubsystem, IntakeConstants.outtakeSpeed)
          .alongWith(new ConveyerCommand(m_ConveyerSubsystem, conveyerConstants.conveyerOutSpeed))
       )
    ); 

    BUTTON_LB_SECONDARY.onFalse(
      new ParallelCommandGroup(
        new IntakeCommand(m_IntakeSubsystem, 0)
        .alongWith(new ConveyerCommand(m_ConveyerSubsystem, 0))
       )
    ); 


    BUTTON_Y_SECONDARY.onTrue(
      new ConveyerCommand(m_ConveyerSubsystem, conveyerConstants.conveyerInSpeed)
    ); 

    BUTTON_Y_SECONDARY.onFalse(
      new ConveyerCommand(m_ConveyerSubsystem, 0)
    ); 
    

    BUTTON_B_SECONDARY.onTrue(
      new shooterVelocityCommand(m_ShooterSubsystem, -ShooterConstants.ampBottomMotorSpeed, -ShooterConstants.ampBottomMotorSpeed)
    ); 

    BUTTON_B_SECONDARY.onFalse(
      new shooterVelocityCommand(m_ShooterSubsystem, 0, 0)
    ); 

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public void resetEncoders(){
    m_DriveSubsystem.resetEncoders();
    m_ShooterSubsystem.resetEncoders(); 
    m_WristSubsystem.resetEncoders();
  }

  public void autonomousInit(){
    m_DriveSubsystem.resetEncoders();
    m_DriveSubsystem.zeroHeading();
    m_WristSubsystem.resetEncoders();
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    autonomousInit();
    return m_autoChooser.getSelected(); 
  }
}

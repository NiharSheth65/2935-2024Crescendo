// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.autoCommands.autoDriveForwardSetDistance;
import frc.robot.commands.autoCommands.autoTurnOnHeadingCommand;
import frc.robot.commands.autoCommands.twoPieceAuto;
import frc.robot.commands.drivetrainCommands.DefaultDriveCommand;
import frc.robot.commands.drivetrainCommands.DriveVelocityControl;
import frc.robot.commands.intakeCommands.IntakeCommand;
import frc.robot.commands.intakeCommands.IntakeVelocityCommand;
import frc.robot.commands.shooterCommands.shootSetSpeedCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
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

  private final Joystick joystick = new Joystick(0); 

  private final JoystickButton BUTTON_A_PRIMARY = new JoystickButton(joystick, OperatorConstants.BUTTON_A_PORT); 
  private final JoystickButton BUTTON_B_PRIMARY = new JoystickButton(joystick, OperatorConstants.BUTTON_B_PORT); 
  private final JoystickButton BUTTON_RB_PRIMARY = new JoystickButton(joystick, OperatorConstants.BUTTON_RB_PORT); 
  private final JoystickButton BUTTON_LB_PRIMARY = new JoystickButton(joystick, OperatorConstants.BUTTON_LB_PORT); 

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);


  private final Command m_twoPieceAuto = new twoPieceAuto(m_DriveSubsystem); 

  SendableChooser<Command> m_autoChooser = new SendableChooser<>(); 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    m_autoChooser.setDefaultOption("two peice", m_twoPieceAuto);
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
    // m_DriveSubsystem.setDefaultCommand(new DriveVelocityControl(m_DriveSubsystem, joystick, 0));
  }
  
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());''
    BUTTON_A_PRIMARY.onFalse(
      new shootSetSpeedCommand(m_ShooterSubsystem, ShooterConstants.shooterOffSpeed, ShooterConstants.shooterOffSpeed)
    );

    BUTTON_A_PRIMARY.onTrue(
        new shootSetSpeedCommand(m_ShooterSubsystem, ShooterConstants.subwooferSpeedTop, ShooterConstants.subwooferSpeedBottom)
        // new shootSetSpeedCommand(m_ShooterSubsystem, IntakeConstants.intakeSpeed, IntakeConstants.intakeSpeed)
    ); 

    BUTTON_B_PRIMARY.toggleOnTrue(
        // new autoDriveForwardSetDistance(m_DriveSubsystem, -50)
        // new autoTurnOnHeadingCommand(m_DriveSubsystem, 39)
        // new secondDriveForwardCommand(m_DriveSubsystem, 10) 
        // new DriveVelocityControl(m_DriveSubsystem, joystick, 5700)
        new shootSetSpeedCommand(m_ShooterSubsystem, -ShooterConstants.subwooferSpeedTop, -ShooterConstants.subwooferSpeedBottom)

      ); 

      BUTTON_B_PRIMARY.toggleOnFalse(
        new shootSetSpeedCommand(m_ShooterSubsystem, 0, 0)
      ); 


    BUTTON_RB_PRIMARY.onTrue(
      new IntakeCommand(m_IntakeSubsystem, IntakeConstants.intakeSpeed) 
      // new IntakeVelocityCommand(m_IntakeSubsystem, IntakeConstants.intakeVelocity)   
    );

    BUTTON_RB_PRIMARY.onFalse(
      // new IntakeVelocityCommand(m_IntakeSubsystem, IntakeConstants.intakeStopVelocity)
      new IntakeCommand(m_IntakeSubsystem, 0)  
    ); 

  
    BUTTON_LB_PRIMARY.onTrue(
      new IntakeCommand(m_IntakeSubsystem, IntakeConstants.outtakeSpeed) 
    ); 

    BUTTON_LB_PRIMARY.onFalse(
      new IntakeCommand(m_IntakeSubsystem, IntakeConstants.intakeStopSpeed)
    );  




  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public void autonomousInit(){
    m_DriveSubsystem.resetEncoders();
    m_DriveSubsystem.zeroHeading();
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    autonomousInit();
    return m_autoChooser.getSelected(); 
  }
}

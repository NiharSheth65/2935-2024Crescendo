// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrainCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveVelocityControl extends Command {
  /** Creates a new DriveVelocityControl. */
  
  private DriveSubsystem driveSubsystem; 
  private Joystick joystick;
  
  private double rightMotorKp = 6e-5; 
  private double rightMotorKi = 0.000000; 
  private double rightMotorKd = 0; 
  private double rightMotorKIz = 0; 
  private double rightMotorKFf = 0.000015; 

  private double leftMotorKp = 6e-5; 
  private double leftMotorKi = 0.000000; 
  private double leftMotorKd = 0; 
  private double leftMotorKIz = 0; 
  private double leftMotorFf= 0.000015; 

  private double leftMax = 1; 
  private double leftMin = -1;
  
  private double rightMax = 1; 
  private double rightMin = -1; 

  private double targetVelocity; 

  private double desiredDrive; 
  private double desiredTurn; 

  public DriveVelocityControl(DriveSubsystem drive, Joystick joy, double velocity) {
    // Use addRequirements() here to declare subsystem dependencies.
   this.joystick = joy; 
   this.driveSubsystem = drive; 
   this.targetVelocity = velocity; 
   addRequirements(driveSubsystem); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.setRightVelocityMode();
    driveSubsystem.setLeftVelocityMode();
    
    driveSubsystem.setLeftPIDF(leftMotorKp, leftMotorKi, leftMotorKd, leftMotorFf);
    driveSubsystem.setRightPIDF(rightMotorKp, rightMotorKi, rightMotorKd, rightMotorKFf);
    driveSubsystem.leftOutPutConstraints(leftMin, leftMax);
    driveSubsystem.rightOutPutConstraints(rightMin, rightMax);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("desired velocity", (-joystick.getRawAxis(OperatorConstants.driveJoystickAxis)) * 5700); 
    SmartDashboard.putNumber("current velocity", driveSubsystem.leftEncoderVelocity());



    if(Math.abs(joystick.getRawAxis(OperatorConstants.driveJoystickAxis)) < DriveConstants.driveDeadband){
      desiredDrive = 0; 
    }
    else{
      desiredDrive = (-joystick.getRawAxis(OperatorConstants.driveJoystickAxis) * 5700); 
    }


    if(Math.abs(joystick.getRawAxis(OperatorConstants.turnJoystickAxis)) < DriveConstants.turnDead){
      desiredTurn = 0; 
    }else{
      desiredTurn  = (-joystick.getRawAxis(OperatorConstants.turnJoystickAxis) * 5700); 
    }
    
    // double desiredVelocity = targetVelocity;
    double currentVelocity = driveSubsystem.leftEncoderVelocity(); 

    driveSubsystem.setVelocity(desiredDrive*1 - desiredTurn*1, desiredDrive*1 + desiredTurn*1);
    // driveSubsystem.setVelocity(leftMax, rightMax);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

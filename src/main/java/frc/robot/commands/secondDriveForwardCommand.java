// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class secondDriveForwardCommand extends Command {

  private DriveSubsystem DRIVE_SUBSYSTEM; 
  private double setPoint; 

  private PIDController drivePID;

  /** Creates a new secondDriveForwardCommand. */
  public secondDriveForwardCommand(DriveSubsystem drive, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.DRIVE_SUBSYSTEM = drive; 
    this.setPoint = distance; 
    this.drivePID = new PIDController(0.2, 0, 0); 
    addRequirements(DRIVE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DRIVE_SUBSYSTEM.resetEncoders(); 
    DRIVE_SUBSYSTEM.zeroHeading(); 
    drivePID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double driveSpeed = drivePID.calculate(DRIVE_SUBSYSTEM.averageEncoderDistanceInInches(), setPoint); 
    
    if(driveSpeed > 0.5){
      driveSpeed = 0.5; 
    }

    DRIVE_SUBSYSTEM.setTank(driveSpeed, driveSpeed);



    SmartDashboard.putNumber("error remaining", setPoint - DRIVE_SUBSYSTEM.averageEncoderDistanceInInches()); 
    SmartDashboard.putNumber("Drive Speed", driveSpeed);

    SmartDashboard.putNumber("Inch travelled", DRIVE_SUBSYSTEM.rightEncoderPosition()*DriveConstants.revToInch);

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

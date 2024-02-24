// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.visionCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class visionDriveCommand extends Command {

  private VisionSubsystem VISION_SUBSYSTEM; 
  private DriveSubsystem DRIVE_SUBSYSTEM; 

  private PIDController driveVisionPID; 
  private PIDController turnVisionPID; 
  
  double gyroTargetPosition; 
  boolean cancelMode = false; 
  double distanceFromLimelightToGoalInches; 
  
  double tvValue; 
  int tvMissedCounter; 
  double lastMeasuredValue; 
  double distanceLast; 

  int setPipelineNumber; 

  double dKp; 
  double dKi; 
  double dKd;

  double tKp; 
  double tKi; 
  double tKd; 

  double driveSetPoint; 

  /** Creates a new visionDriveCommand. */
  public visionDriveCommand(DriveSubsystem drive, VisionSubsystem vision, boolean cancel, int pipelineNumber, double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    this.VISION_SUBSYSTEM = vision; 
    this.DRIVE_SUBSYSTEM = drive; 
    this.driveSetPoint = setPoint; 

    if(pipelineNumber == 0){
      dKp = 0.05; 
      dKi = 0; 
      dKd = 0; 

      tKp = 0.02; 
      tKi = 0; 
      tKd = 0; 
    }

    else if(pipelineNumber == 1){
      dKp = 0.02;
      dKi = 0.0;
      dKd = 0;

      tKp = 0.01; 
      tKi = 0; 
      tKd = 0; 
    }
    // this.driveVisionPID = new PIDController(0.0175, 0, 0); 
    this.driveVisionPID = new PIDController(dKp, dKi, dKd); 
    this.turnVisionPID = new PIDController(tKp, tKi, tKd); 
    
    this.cancelMode = cancel; 
    this.setPipelineNumber = pipelineNumber; 

    addRequirements(VISION_SUBSYSTEM);
    addRequirements(DRIVE_SUBSYSTEM);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    driveVisionPID.reset();
    turnVisionPID.reset();
    gyroTargetPosition = DRIVE_SUBSYSTEM.getYaw(); 
    tvMissedCounter = 0; 
    VISION_SUBSYSTEM.setPipeline(setPipelineNumber);
    VISION_SUBSYSTEM.setLED(0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double limelightLensHeight = 18; 
    double goalHeight; 
    double limelightMountAngle = -20; 

    if(setPipelineNumber == 0){
      goalHeight = 0; 
    }
    else if(setPipelineNumber == 1){
      goalHeight = 22.5; 
    }else{
      goalHeight = 0; 
    }

    double angleToGoalDegrees = limelightMountAngle + VISION_SUBSYSTEM.ty(); 
    double angleToGoalRadians = Math.toRadians(angleToGoalDegrees); 

    if(VISION_SUBSYSTEM.tv() == 0){
      tvMissedCounter++; 
    }

    
    else if(VISION_SUBSYSTEM.tv() == 1){
      tvMissedCounter = 0;  
      distanceFromLimelightToGoalInches = (goalHeight - limelightLensHeight)/Math.tan(angleToGoalRadians); 
      distanceLast = distanceFromLimelightToGoalInches; 
    }
    

    double driveSpeed = driveVisionPID.calculate(distanceFromLimelightToGoalInches, driveSetPoint); 
    double turnSpeed = turnVisionPID.calculate(DRIVE_SUBSYSTEM.getYaw(), gyroTargetPosition); 

    if(driveSpeed > 1.00){
      driveSpeed = 1.00; 
    }

    else if(driveSpeed < -1.00){
      driveSpeed = -1.00; 
    }

    // driveSubsystem.tankMode(turnSpeed, -1*turnSpeed);
    DRIVE_SUBSYSTEM.setTank(-driveSpeed - turnSpeed, -driveSpeed + turnSpeed);

    SmartDashboard.putNumber("distance to note", distanceFromLimelightToGoalInches); 
    SmartDashboard.putNumber("vision drive speed", driveSpeed); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DRIVE_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(cancelMode == true){
      return true; 
    }

    else if(Math.abs(driveSetPoint - distanceFromLimelightToGoalInches) < 2 && tvMissedCounter == 0){
      return true; 
    }

    else{
      return false; 
    }
  }
}

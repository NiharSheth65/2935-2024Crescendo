// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.photonvisionCommands;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.photonVisionConstants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonvisionSubsystem;

public class photonVisionDetermineDistance extends Command {

  private DriveSubsystem DRIVE_SUBSYSTEM; 
  private PhotonvisionSubsystem PHOTON_SUBSYSTEM; 

  private double distanceSetpoint; 
  private double alignmentSetpoint; 

  private final String VISION_PREFIX = "Vision/"; 

  private PIDController driveController; 

  private double driveKP = 1.0;  
  private double driveKI = 0.0; 
  private double driveKD = 0;

  double pitch; 
  double driveSpeed; 

  private boolean end; 

  private double initTime; 
  private double initInitTime; 

  private int targetId;
  private double bestTargetRange; 

  private SlewRateLimiter driveLimiter = new SlewRateLimiter(DriveConstants.driveSlew); 


  private int pitchMissed; 

  private double error; 


  private double bestTargetPitch; 
  /** Creates a new photonVisionDetermineDistance. */
  public photonVisionDetermineDistance(PhotonvisionSubsystem photon, DriveSubsystem drive, boolean endCommand, int desiredTagId) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.DRIVE_SUBSYSTEM = drive; 
    this.PHOTON_SUBSYSTEM = photon; 
    this.end = endCommand; 
    this.targetId = desiredTagId; 

    this.driveController = new PIDController(driveKP, driveKI, driveKD); 

    addRequirements(DRIVE_SUBSYSTEM);
    addRequirements(PHOTON_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveController.reset(); 
    pitch = 0; 
    initInitTime = System.currentTimeMillis(); 
    pitchMissed = 0;
    bestTargetPitch = 0;
    error = 0; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    var result = PHOTON_SUBSYSTEM.getLatestResult(); 

 
    if(result.hasTargets()){

      PhotonTrackedTarget bestTarget = null;

            // Loop through detected targets to find the AprilTag with the desired ID
      for (var target : result.getTargets()) {
          if (target.getFiducialId() == targetId) {
              bestTarget = target;
              break; // Stop searching once we've found our target
          }
      }

      if (bestTarget != null) {
        // Calculate alignment based on the target's position
        pitch = bestTarget.getPitch();
        bestTargetPitch = pitch; 
        pitchMissed = 0;

        initTime = System.currentTimeMillis(); 
     }

     else{
      initTime = initInitTime;
     }

    }else{
      initTime = initInitTime;
      pitchMissed++; 
    }

    driveSpeed = -driveController.calculate(pitch, 0); 

    if(driveSpeed > photonVisionConstants.photonMaxDriveSpeed){
      driveSpeed = photonVisionConstants.photonMaxDriveSpeed; 
    }

    else if(driveSpeed < -photonVisionConstants.photonMaxDriveSpeed){
      driveSpeed = -photonVisionConstants.photonMaxDriveSpeed; 
    }

    SmartDashboard.putNumber("drive speed", driveSpeed);     
    SmartDashboard.putNumber("pitch-photon", pitch);
    SmartDashboard.putNumber("pithc missed counter", pitchMissed); 

    DRIVE_SUBSYSTEM.setTank(driveLimiter.calculate(driveSpeed), driveLimiter.calculate(driveSpeed));
    SmartDashboard.putNumber("drive missed counter",  pitchMissed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pitchMissed = 0;
    DRIVE_SUBSYSTEM.stop(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(end == true){
      return true; 
    }

    else if(bestTargetPitch > 0){
      return true; 
    }

    else if(pitchMissed > 100){
      return true; 
    }

    else if(Math.abs(System.currentTimeMillis() - initTime) > 5000){
      return true; 
    }

    else if(Math.abs(System.currentTimeMillis() - initInitTime) > 5000){
      return true; 
    }

    else{
      return false; 
    }
  }
}

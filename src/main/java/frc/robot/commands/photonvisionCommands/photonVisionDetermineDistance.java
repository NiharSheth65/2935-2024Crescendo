// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.photonvisionCommands;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.photonVisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonvisionSubsystem;

public class photonVisionDetermineDistance extends Command {

  private DriveSubsystem DRIVE_SUBSYSTEM; 
  private PhotonvisionSubsystem PHOTON_SUBSYSTEM; 

  private double distanceSetpoint; 
  private double alignmentSetpoint; 

  private final String VISION_PREFIX = "Vision/"; 

  // private PIDController driveController; 

  // private double driveKP = 1.0;  
  // private double driveKI = 0.0; 
  // private double driveKD = 0;

  double pitch; 
  double driveSpeed; 

  private boolean end; 

  private double range; 
  private double GOAL_RANGE_METERS; 

  private double initTime; 
  private double initInitTime; 

  private int targetId;
  private double bestTargetRange; 

  private int rangeCounter; 
  private double totalDistance; 

  // private SlewRateLimiter driveLimiter = new SlewRateLimiter(DriveConstants.driveSlew); 
  /** Creates a new photonVisionDetermineDistance. */
  public photonVisionDetermineDistance(PhotonvisionSubsystem photon, DriveSubsystem drive, boolean endCommand, int desiredTagId) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.DRIVE_SUBSYSTEM = drive; 
    this.PHOTON_SUBSYSTEM = photon; 
    this.end = endCommand; 
    this.targetId = desiredTagId; 

    // this.driveController = new PIDController(driveKP, driveKI, driveKD); 

    addRequirements(DRIVE_SUBSYSTEM);
    addRequirements(PHOTON_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // driveController.reset(); 
    pitch = 0; 
    rangeCounter = 0; 
    initInitTime = System.currentTimeMillis(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    final double CAMERA_HEIGHT_METERS = (photonVisionConstants.cameraHeight);
    final double TARGET_HEIGHT_METERS = (photonVisionConstants.speakerHeight);
    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = (photonVisionConstants.cameraMountAngle);

    GOAL_RANGE_METERS = Units.feetToMeters(3);

    var result = PHOTON_SUBSYSTEM.getLatestResult(); 

 
    if(result.hasTargets()){

      PhotonTrackedTarget bestTarget = null;

            // Loop through detected targets to find the AprilTag with the desired ID
      for (var target : result.getTargets()) {
          if (target.getFiducialId() == targetId) {
              bestTarget = target;
              rangeCounter++; 
              break; // Stop searching once we've found our target
          }
      }

      if (bestTarget != null) {
        // Calculate alignment based on the target's position
        pitch = bestTarget.getPitch();

        initTime = System.currentTimeMillis();

        range =
                            PhotonUtils.calculateDistanceToTargetMeters(
                                    CAMERA_HEIGHT_METERS,
                                    TARGET_HEIGHT_METERS,
                                    CAMERA_PITCH_RADIANS,
                                    Units.degreesToRadians(pitch));
        
        bestTargetRange = range; 
        range += totalDistance; 
     }

     else{
      initTime = initInitTime;
     }

    }else{
      range = GOAL_RANGE_METERS; 
      initTime = initInitTime;
    }


    // driveSpeed = driveController.calculate(range, GOAL_RANGE_METERS); 

    // if(driveSpeed > photonVisionConstants.photonMaxDriveSpeed){
    //   driveSpeed = photonVisionConstants.photonMaxDriveSpeed; 
    // }

    // else if(driveSpeed < -photonVisionConstants.photonMaxDriveSpeed){
    //   driveSpeed = -photonVisionConstants.photonMaxDriveSpeed; 
    // }

    SmartDashboard.putNumber("distance", range); 
    SmartDashboard.putNumber("drive speed", driveSpeed); 
    // DRIVE_SUBSYSTEM.setTank(driveLimiter.calculate(driveSpeed), driveLimiter.calculate(driveSpeed));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DRIVE_SUBSYSTEM.stop(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(end == true){
      return true; 
    }


    // else if(Math.abs(GOAL_RANGE_METERS - bestTargetRange) < 0.15){
    //   return true; 
    // }

    else if(rangeCounter > 10){
      SmartDashboard.putNumber("average range", (totalDistance/rangeCounter)); 
      return true; 
    }

    else if(Math.abs(System.currentTimeMillis() - initTime) > 1000){
      return true; 
    }

    else if(Math.abs(System.currentTimeMillis() - initInitTime) > 3000){
      return true; 
    }

    else{
      return false; 
    }
  }
}

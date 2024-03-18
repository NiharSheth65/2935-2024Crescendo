// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.photonvisionCommands;

import javax.swing.plaf.basic.BasicInternalFrameTitlePane.SystemMenuBar;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.photonVisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonvisionSubsystem;

public class photonVisionDriveAndAlignCommand extends Command {

  private DriveSubsystem DRIVE_SUBSYSTEM; 
  private PhotonvisionSubsystem PHOTON_SUBSYSTEM; 

  private double distanceSetpoint; 
  private double alignmentSetpoint; 

  private final String VISION_PREFIX = "Vision/"; 

  private PIDController forwardController; 
  private PIDController turnController; 

  private double driveKP = 0.4; 
  private double driveKI = 0; 
  private double driveKD = 0;
  
  private double turnKP = 0.2;  
  private double turnKI = 0.0; 
  private double turnKD = 0;

  
  
  double yaw; 
  double rotationSpeed; 
  double forwardSpeed; 

  private boolean end; 

  private double range; 
  private double GOAL_RANGE_METERS; 

  private double initTime; 
  private double initInitTime; 

  private int targetId;

  private double bestTargetYaw; 

  private SlewRateLimiter turnLimiter = new SlewRateLimiter(DriveConstants.turnSlew); 

  /** Creates a new photonVisionDriveAndAlignCommand. */
  public photonVisionDriveAndAlignCommand(PhotonvisionSubsystem photon, DriveSubsystem drive, double distanceTo, double angleTo, boolean endCommand, int desiredTagId) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.distanceSetpoint = distanceTo; 
    this.alignmentSetpoint = angleTo; 
    this.DRIVE_SUBSYSTEM = drive; 
    this.PHOTON_SUBSYSTEM = photon; 
    this.end = endCommand; 
    this.targetId = desiredTagId; 


    this.forwardController = new PIDController(driveKP, driveKI, driveKD); 
    this.turnController = new PIDController(turnKP, turnKI, turnKD); 

    addRequirements(DRIVE_SUBSYSTEM);
    addRequirements(PHOTON_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    forwardController.reset();
    turnController.reset();
    rotationSpeed = 0; 
    yaw = 0; 
    bestTargetYaw = 0; 

    initInitTime = System.currentTimeMillis(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    final double CAMERA_HEIGHT_METERS = (photonVisionConstants.cameraHeight);
    final double TARGET_HEIGHT_METERS = (photonVisionConstants.speakerHeight);
    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = (photonVisionConstants.cameraMountAngle);

    GOAL_RANGE_METERS = Units.feetToMeters(4);

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
        yaw = bestTarget.getYaw();
        bestTargetYaw = yaw; 

        // Logic to align the robot using xOffset
        // For simplicity, just printing out the offset here
        // Use xOffset to control your robot's drivetrain here
        // This will depend on your robot's specific drivetrain and control system

        initTime = System.currentTimeMillis();

        range =
                            PhotonUtils.calculateDistanceToTargetMeters(
                                    CAMERA_HEIGHT_METERS,
                                    TARGET_HEIGHT_METERS,
                                    CAMERA_PITCH_RADIANS,
                                    Units.degreesToRadians(result.getBestTarget().getPitch()));


        forwardSpeed = forwardController.calculate(range, GOAL_RANGE_METERS); 
     }
     else{
      initTime = initInitTime;
     }
    }else{
      yaw = 0; 
      // rotationSpeed = 0; 
      forwardSpeed = 0;
      initTime = initInitTime;
    }


    rotationSpeed = turnController.calculate(yaw, 0);

    if(rotationSpeed > photonVisionConstants.photonMaxTurnSpeed){
      rotationSpeed = photonVisionConstants.photonMaxTurnSpeed; 
    }

    else if(rotationSpeed < -photonVisionConstants.photonMaxTurnSpeed){
      rotationSpeed = -photonVisionConstants.photonMaxTurnSpeed; 
    }


    DRIVE_SUBSYSTEM.setTank(turnLimiter.calculate(rotationSpeed), -turnLimiter.calculate(rotationSpeed));

    SmartDashboard.putNumber(VISION_PREFIX + "photon-please-work-yaw", yaw); 

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


    else if(Math.abs(bestTargetYaw - 0) < 5){
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

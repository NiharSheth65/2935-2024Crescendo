// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.photonvisionCommands;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

  private double driveKP = photonVisionConstants.driveKp; 
  private double driveKI = photonVisionConstants.driveKi; 
  private double driveKD = photonVisionConstants.driveKd;
  
  private double turnKP = photonVisionConstants.turnKp; 
  private double turnKI = photonVisionConstants.turnKi; 
  private double turnKD = photonVisionConstants.turnKd;
  
  double yaw; 
  double rotationSpeed; 
  double forwardSpeed; 

  private boolean end; 

  private double range; 
  private double GOAL_RANGE_METERS; 

  private double initTime; 

  /** Creates a new photonVisionDriveAndAlignCommand. */
  public photonVisionDriveAndAlignCommand(PhotonvisionSubsystem photon, DriveSubsystem drive, double distanceTo, double angleTo, boolean endCommand) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.distanceSetpoint = distanceTo; 
    this.alignmentSetpoint = angleTo; 
    this.DRIVE_SUBSYSTEM = drive; 
    this.PHOTON_SUBSYSTEM = photon; 
    this.end = endCommand; 


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

    initTime = System.currentTimeMillis(); 
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

      range =
                        PhotonUtils.calculateDistanceToTargetMeters(
                                CAMERA_HEIGHT_METERS,
                                TARGET_HEIGHT_METERS,
                                CAMERA_PITCH_RADIANS,
                                Units.degreesToRadians(result.getBestTarget().getPitch()));

     yaw = result.getBestTarget().getYaw(); 
     rotationSpeed = turnController.calculate(yaw, 0);


    if(rotationSpeed > VisionConstants.limelightTurnSpeedLimit){
      rotationSpeed = VisionConstants.limelightTurnSpeedLimit; 
    }

    else if(rotationSpeed < -VisionConstants.limelightTurnSpeedLimit){
      rotationSpeed = -VisionConstants.limelightTurnSpeedLimit; 
    }
     forwardSpeed = forwardController.calculate(range, GOAL_RANGE_METERS); 
    }

    else{
      yaw = 1000; 
      rotationSpeed = 0; 
      forwardSpeed = 0;
    }

    DRIVE_SUBSYSTEM.setTank(rotationSpeed, -rotationSpeed);

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

    else if(Math.abs(yaw - 0) < 7){
      return true; 
    }

    else if(Math.abs(System.currentTimeMillis() - initTime) > 3000){
      return true; 
    }

    else{
      return false; 
    }
  
  }
}

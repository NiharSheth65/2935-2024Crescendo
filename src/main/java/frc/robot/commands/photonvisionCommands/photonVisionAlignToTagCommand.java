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

public class photonVisionAlignToTagCommand extends Command {

  private DriveSubsystem DRIVE_SUBSYSTEM; 
  private PhotonvisionSubsystem PHOTON_SUBSYSTEM; 

  private double distanceSetpoint; 
  private double alignmentSetpoint; 

  private final String VISION_PREFIX = "Vision/"; 

  private PIDController turnController; 

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


  private SlewRateLimiter turnLimiter = new SlewRateLimiter(DriveConstants.turnSlew); 


  /** Creates a new photonVisionAlignToTagCommand. */
  public photonVisionAlignToTagCommand(PhotonvisionSubsystem photon, DriveSubsystem drive, double distanceTo, double angleTo, boolean endCommand) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.distanceSetpoint = distanceTo; 
    this.alignmentSetpoint = angleTo; 
    this.DRIVE_SUBSYSTEM = drive; 
    this.PHOTON_SUBSYSTEM = photon; 
    this.end = endCommand; 

    this.turnController = new PIDController(turnKP, turnKI, turnKD); 

    addRequirements(DRIVE_SUBSYSTEM);
    addRequirements(PHOTON_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnController.reset();
    rotationSpeed = 0; 
    yaw = 0; 

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

    if (result.hasTargets()) {
        yaw = result.getBestTarget().getYaw(); 
    } else {
        // If we have no targets, stay still.
        rotationSpeed = 0;
        yaw = 0; 
    }
  
    rotationSpeed = turnController.calculate(yaw, 0);

    if(rotationSpeed > photonVisionConstants.photonMaxTurnSpeed){
      rotationSpeed = photonVisionConstants.photonMaxTurnSpeed; 
    }

    else if(rotationSpeed < -photonVisionConstants.photonMaxTurnSpeed){
      rotationSpeed = -photonVisionConstants.photonMaxTurnSpeed; 
    }


    DRIVE_SUBSYSTEM.setTank(turnLimiter.calculate(rotationSpeed), -turnLimiter.calculate(rotationSpeed));

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

    else{
      return false; 
    }
  }
}

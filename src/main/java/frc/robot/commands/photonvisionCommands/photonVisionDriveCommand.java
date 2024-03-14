// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.photonvisionCommands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.photonVisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonvisionSubsystem;

public class photonVisionDriveCommand extends Command {

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

  boolean end; 


  double range; 

  
  /** Creates a new photonVisionDriveCommand. */
  public photonVisionDriveCommand(PhotonvisionSubsystem photon, DriveSubsystem drive, double distanceTo, double angleTo, boolean endCommand) {
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double CAMERA_HEIGHT_METERS = (photonVisionConstants.cameraHeight);
    final double TARGET_HEIGHT_METERS = (photonVisionConstants.speakerHeight);

    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = (photonVisionConstants.cameraMountAngle);

    // How far from the target we want to be
    final double GOAL_RANGE_METERS = Units.feetToMeters(distanceSetpoint);

    // Change this to match the name of your camera
    // PID constants should be tuned per robot

    var result = PHOTON_SUBSYSTEM.getLatestResult(); 

 
    if(result.hasTargets()){
      range =
                        PhotonUtils.calculateDistanceToTargetMeters(
                                CAMERA_HEIGHT_METERS,
                                TARGET_HEIGHT_METERS,
                                CAMERA_PITCH_RADIANS,
                                Units.degreesToRadians(result.getBestTarget().getPitch()));

                // Use this range as the measurement we give to the PID controller.
                // -1.0 required to ensure positive PID controller effort _increases_ range
                forwardSpeed = forwardController.calculate(range, GOAL_RANGE_METERS);
              SmartDashboard.putNumber(VISION_PREFIX + "photon-distance", range); 
    }

    else{
      rotationSpeed = 0; 
      forwardSpeed = 0;
    }

    if(forwardSpeed > 0.35){
      forwardSpeed = 0.35; 
    }

    else if(forwardSpeed < -0.35){
      forwardSpeed = -0.35;
    }

    else{
      forwardSpeed = forwardSpeed; 
    }

    DRIVE_SUBSYSTEM.setTank(forwardSpeed*2.0, forwardSpeed*2.0);


    SmartDashboard.putNumber(VISION_PREFIX + "photon-drive-speed", forwardSpeed); 

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

    else if(Math.abs(range) < 1.75){
      return true; 
    }

    else{
      return false; 
    }
  }
}

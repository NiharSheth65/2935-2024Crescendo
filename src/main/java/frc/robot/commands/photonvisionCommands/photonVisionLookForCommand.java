// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.photonvisionCommands;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.photonVisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonvisionSubsystem;

public class photonVisionLookForCommand extends Command {

  private PhotonvisionSubsystem PHOTON_SUBSYSTEM; 



  private final String VISION_PREFIX = "Vision/"; 


  
  double yaw; 

  private boolean end; 

  private double initTime; 
  private double initInitTime; 

  private int targetId;

  private boolean targetFound; 

  /** Creates a new photonVisionLookForCommand. */
  public photonVisionLookForCommand(PhotonvisionSubsystem photon, boolean endCommand, int desiredTagId) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.PHOTON_SUBSYSTEM = photon; 
    this.end = endCommand; 
    this.targetId = desiredTagId; 

    addRequirements(PHOTON_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    initInitTime = System.currentTimeMillis(); 
    targetFound = false; 
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
        yaw = bestTarget.getYaw();

        targetFound = true; 
     }
     else{
      initTime = initInitTime;
      // targetFound = false; 
     }
    }else{
      targetFound = false; 
      initTime = initInitTime;
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(end == true){
      return true; 
    }

    else if(targetFound == true){
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

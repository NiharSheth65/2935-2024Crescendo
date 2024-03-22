// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.photonVisionConstants;

public class PhotonvisionSubsystem extends SubsystemBase {
  /** Creates a new PhotonvisionSubsystem. */

  // private PhotonCamera camera = new PhotonCamera("photonvision"); 

  private PhotonCamera photonCamera; 

  public PhotonvisionSubsystem() {
    photonCamera = new PhotonCamera("USB_Camera"); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run  
  }

  public PhotonPipelineResult getLatestResult() {
    return photonCamera.getLatestResult();
  }


  // public PhotonPipelineResult latestResult(){
  //   return camera.getLatestResult(); 
  // }

  // public double rangeToTarget(double goalHeight){
  //   return PhotonUtils.calculateDistanceToTargetMeters(photonVisionConstants.cameraHeight, goalHeight, photonVisionConstants.cameraHeight, getYaw()); 
  // }

  // public double getPitch(){
  //   return Units.degreesToRadians(latestResult().getBestTarget().getPitch()); 
  // }

  // public double getYaw(){
  //   return latestResult().getBestTarget().getYaw(); 
  // }




}

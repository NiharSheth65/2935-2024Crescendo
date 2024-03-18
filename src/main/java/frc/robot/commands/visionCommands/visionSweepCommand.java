// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.visionCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class visionSweepCommand extends Command {

  private VisionSubsystem VISION_SUBSYSTEM; 

  private PIDController visionPID; 
  
  boolean endCommand; 
  boolean targetFound; 

  double gyroValue; 
  double error; 

  double tvValue; 
  int tvMissedCoutner; 
  double lastMeasuredValue; 

  int setPipelineNumber; 

  double  kp; 
  double ki; 
  double kd; 

  int targetToTrack; 

  double turnTolerance; 

  /** Creates a new visionSweepCommand. */
  public visionSweepCommand(VisionSubsystem vision, int pipeline, boolean end, double tolerance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.VISION_SUBSYSTEM = vision; 
    // Use addRequirements() here to declare subsystem dependencies.
    
    if(pipeline == 0){
      kp = 0.035; 
      ki = 0.02; 
      kd = 0;   
    }

    
    else if(pipeline == 1){
      kp = 0.075; 
      ki = 0.025; 
      kd = 0;
    }


    this.visionPID = new PIDController(kp, ki, kd); 
    this.endCommand = end;  
    this.setPipelineNumber = pipeline;
    this.turnTolerance = tolerance;  

    addRequirements(VISION_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    visionPID.reset();
    tvMissedCoutner = 0; 
    VISION_SUBSYSTEM.setPipeline(setPipelineNumber);
    VISION_SUBSYSTEM.setLED(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     int numberOfTargets = (int) VISION_SUBSYSTEM.limelighTableRead().getEntry("tv").getDouble(0);

    if(numberOfTargets > 0){
      double bestTargetArea = 0.0; 
      targetToTrack = 0; 

      for(int i=0; i < numberOfTargets; i++){
        double targetArea = VISION_SUBSYSTEM.targetArea(i); 

        if(targetArea > bestTargetArea){
          bestTargetArea = targetArea; 
          targetArea = i; 
        }
      }
    }

    double targetValue = 0; 
    double measuredValue = VISION_SUBSYSTEM.tx(); 
    tvValue = VISION_SUBSYSTEM.tv(); 

    if(tvValue == 0){
      tvMissedCoutner++;
      targetFound = false; 
    }

    else{
      tvMissedCoutner = 0;
      lastMeasuredValue = VISION_SUBSYSTEM.tx(); 
      targetFound = true; 
    }

    SmartDashboard.putBoolean("target found", targetFound); 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(targetFound == true){
    //   return true; 
    // }

    if(endCommand == true){
      return true; 
    }
    
    else{
      return false; 
    }
  }
}

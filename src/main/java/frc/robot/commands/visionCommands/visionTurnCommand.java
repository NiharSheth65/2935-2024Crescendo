// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.visionCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.autoCommands.threeGPAuto;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class visionTurnCommand extends Command {

  private VisionSubsystem VISION_SUBSYSTEM; 
  private DriveSubsystem DRIVE_SUBSYSTEM; 

  private PIDController visionPID; 
  
  boolean turnBool; 
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

  double alignmentTolerance; 

  /** Creates a new visionTurnCommand. */
  public visionTurnCommand(DriveSubsystem drive, VisionSubsystem vision, int pipeline, boolean turn, double tolerance) {
    
    this.DRIVE_SUBSYSTEM = drive; 
    this.VISION_SUBSYSTEM = vision; 

    this.visionPID = new PIDController(kp, ki, kd); 
    this.turnBool = turn; 
    this.setPipelineNumber = pipeline; 
    this.alignmentTolerance = tolerance;


    // Use addRequirements() here to declare subsystem dependencies.
    
    if(pipeline == 0){
      kp = 0.02; 
      ki = 0.025; 
      kd = 0;   
    }

    else if(pipeline == 1){
      kp = 0.03; 
      ki = 0.025; 
      kd = 0;
    }


    addRequirements(DRIVE_SUBSYSTEM);
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
    }

    else{
      tvMissedCoutner = 0;
      lastMeasuredValue = VISION_SUBSYSTEM.tx(); 
    }

    double outputSpeed = visionPID.calculate(measuredValue, targetValue); 

    if(outputSpeed > 0.8){
      outputSpeed = 0.8; 
    }

    else if(outputSpeed < -0.8){
      outputSpeed = -0.8; 
    }

    DRIVE_SUBSYSTEM.setTank(outputSpeed, -outputSpeed);

    error = measuredValue - targetValue; 
    
    SmartDashboard.putNumber("align error", error); 
    SmartDashboard.putNumber("align speed", outputSpeed); 
    SmartDashboard.putNumber("number of targets", numberOfTargets); 
    boolean complete = false; 

    if(Math.abs(error) < 0.25){
      complete = true; 
      // turnBool = true; 
    }
    else{
      complete = false;
    }

    SmartDashboard.putBoolean("alignment", complete); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if(turnBool == true){
      return true; 
    }
    
    else if(Math.abs(error) < alignmentTolerance && tvMissedCoutner == 0){
      return true; 
    }
    
    else{
      return false; 
    }
  }
}

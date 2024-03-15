// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    
    public static final int primaryControllerPort = 0; 
    public static final int secondaryControllerPort = 1; 

    public static final int BUTTON_A_PORT = 1;
    public static final int BUTTON_B_PORT = 2;
    public static final int BUTTON_X_PORT = 3;
    public static final int BUTTON_Y_PORT = 4;

    public static final int BUTTON_RB_PORT = 6;
    public static final int BUTTON_LB_PORT = 5;

    public static final int BUTTON_START = 8;
    public static final int BUTTON_RIGHT_JOYSTICK_PORT = 9;
    public static final int BUTTON_LEFT_JOYSTICK_PORT = 10;
  
    public static final int driveJoystickAxis = 1; 
    public static final int turnJoystickAxis = 4; 

    public static final int rightTriggerAxis = 3; 
    public static final int leftTriggerAxis = 2; 

    public static final double triggerThreshold = 0.5; 


  }

  public static class DriveConstants{
    public static final int leftFrontMotorId = 1; 
    public static final int leftBackMotorId = 2; 
    public static final int rightFrontMotorId = 3; 
    public static final int rightBackMotorId = 4;

    public static final double driveSlew = 5; 
    public static final double turnSlew = 2;
    
    public static final double driveSlowSpeed = 0.75; 
    public static final double driveFastSpeed = 1; 

    public static final double turnSlowSpeed = 0.5; 
    public static final double turnFastSpeed = 0.75; 

    public static final double driveDeadband = 0.1; 
    public static final double turnDead = 0.1; 

    public static final double encoderToInchConversion = 1/42 * ((Math.PI * 4) / 5.95);
    
    public static final double inchToRev = (5.95/12.56); 
    public static final double revToInch = (12.56/5.95); 

    public static final int rightDirection = 0; 
    public static final int leftDirection = 1; 
    
    // public static final double autoDriveForwardAndIntakeDistance = VisionConstants.desiredApproachDistance + 3; 
    public static final double autoDriveForwardAndIntakeDistance = 35; 

    public static final double autoDriveSpeed = 0.60; 
    public static final double autoDriveLimelightSpeed = 0.45; 
  }

  public static class ShooterConstants{
    public static final int topShooterMotorId = 6; 
    public static final int bottomShooterMotorId = 5; 

    public static final double shooterFullSpeed = 1.0; 
    public static final double shooterOffSpeed = 0.0; 

    // public static final double topSpeed = 0.26;
    // public static final double bottomSpeed = 0.28;


    // public static final double subwooferSpeedTop = 0.50; 
    // public static final double subwooferSpeedBottom = 0.50; 
    
    public static final double speakerTopMotorSpeed = -5700*0.80; 
    public static final double speakerBottomMotorSpeed = 5700*0.80; 

        
    public static final double speakerAutoTopMotorSpeed = -5700*0.75; 
    public static final double speakerAutoBottomMotorSpeed = 5700*0.35; 
  
  
    public static final double speakerTopMotorSpeedAutoLine = -5700*0.9; 
    public static final double speakerBottomMotorSpeedAutoLine = 5700*0.4; 


    public static final double autoCentreTopMotorSpeed = -5700 * 0.50; 
    public static final double autoCentreBottomMotorSpeed = 5700 * 0.50; 

    // public static final double ampTopMotorSpeed = 5700 * 0.1; 
    // public static final double ampBottomMotorSpeed = 5700 * 0.35; 

    public static final double shooterHoldInSpeed = 5700 * -0.1; 
    public static final double ampTopMotorSpeed = 5700 * -0.2; 
    public static final double ampBottomMotorSpeed = 5700 * 0.6; 

    public static final double shooterStopSpeed = 0; 
  }

  public static class IntakeConstants{
    public static final int intakeMotorId = 9;
    public static final double intakeSpeed = 0.75; 
    public static final double intakeVelocity = 0.10 * 11000; 
    public static final double outtakeVelocity = -0.10 * 11000; 
    public static final double intakeStopSpeed = 0; 
    public static final double intakeStopVelocity = 0; 
    public static final double outtakeSpeed = -0.45;
    
    public static final int switchOnePort = 0; 
    public static final int switchTwoPort = 1; 
    public static final int switchThreePort = 2; 
  }

  public static class VisionConstants{
    public static final double limlightLensHeight = 17.5; 
    public static final double shooterAprilTagHeight = 16.5; 
    public static final double limelightMountAngle = -6; 

    public static final double roughAlignmentTolerance = 5.5; 
    public static final double fineAlighnmentTolerance = 3.5; 

    public static final double desiredApproachDistance = 40; 

    public static final double limelightTurnSpeedLimit = 0.30; 
  }


  public static class WristConstants{
    public static final int wristMotorId = 10; 


    public static final double wristKP = 0.05; 
    public static final double wristKI = 0.0; 
    public static final double wristKD = 0;

    public static final double wristSpeedTolerance = 0.65; 
    public static final double wristTolerance = 0.80; 
    public static final double wristMaxSpeed = 1.0; 

    public static final double wristHomePosition = 0; 
    public static final double wristfeedShooterPosition = 0; 
    public static final double wristIntakePosition = 40; 
  }

  public static class conveyerConstants{
      public static final int conveyerTopMotorId = 7;
      public static final int conveyerBottomMotorId = 8;
      public static final double conveyerInSpeed = 1.0; 
      public static final double conveyerOutSpeed = -1.0; 
  }

  public static class photonVisionConstants{

    public static double cameraHeight = Units.inchesToMeters(22); 
    public static double cameraMountAngle = Units.degreesToRadians(45); 

    public static double ampHeight = Units.inchesToMeters(0); 
    public static double speakerHeight = Units.inchesToMeters(57); 
    public static double sourceHeight = Units.inchesToMeters(0); 
    public static double trapHeight = Units.inchesToMeters(0); 
  
    public static double ampDistance = Units.inchesToMeters(20); 
    public static double speakerDistance = Units.inchesToMeters(0); 
    public static double sourceDistance = Units.inchesToMeters(0); 
    public static double trapDistance = Units.inchesToMeters(0); 

    public static double driveKp = 0.4; 
    public static double driveKi = 0; 
    public static double driveKd = 0; 

    public static double turnKp = 0.03; 
    public static double turnKi = 0.025; 
    public static double turnKd = 0; 

    
    public static int sourceLeftBlueID = 1; 
    public static int sourceRightBlueID = 2; 

    public static int speakerMiddleRedID = 4;
    public static int speakerOffCentreRedID = 3;

    public static int ampRedID = 5; 
    public static int ampBlueID = 6;

    public static int speakerMiddleBlueID = 7;
    public static int speakerOffCentreBlueID = 8;

    public static int sourceLeftRedID = 9; 
    public static int sourceRightRedID = 10; 

    public static int stageRedLeftID = 11; 
    public static int stageRedRightID = 12; 
    public static int stageRedCentreID = 13; 

    public static int stageBlueLeftID = 14; 
    public static int stageBlueRightID = 15; 
    public static int stageBlueCentreID = 16; 

    public static double photonMaxTurnSpeed = 0.3; 

  }

  public static class autoConstants{
    public static int nonLightNotePipeline = 0; 
    public static int withLightNotePipeline = 1; 

    public static double gyroTurnMaxSpeed = 0.6; 
  }

  public static class LedConstants{
    public static final int ledPort = 0; 
    public static final int truckLightPort = 3; 
    
    public static final int ledLength = 60; 
    
    public static final int[] greenColourCode = {0, 255, 0}; 
    public static final int[] blueColourCode = {0, 0, 255}; 
    public static final int[] redColourCode = {255, 0, 0}; 
    public static final int[] orangeColourCode = {255, 25, 0}; 

    public static final int[] whiteColourCode = {255, 125, 50}; 
    public static final int[] vermillionColourCode = {255, 255, 255}; 
    // public static final int policeFlashCycle = 1000; 
  
  }

}

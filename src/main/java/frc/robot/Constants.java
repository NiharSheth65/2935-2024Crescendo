// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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


  }

  public static class DriveConstants{
    public static final int leftFrontMotorId = 1; 
    public static final int leftBackMotorId = 2; 
    public static final int rightFrontMotorId = 3; 
    public static final int rightBackMotorId = 4;

    public static final double driveSlew = 5; 
    public static final double turnSlew = 2;
    
    public static final double driveSlowSpeed = 0.5; 
    public static final double driveFastSpeed = 1; 

    public static final double turnSpeed = 0.5; 

    public static final double driveDeadband = 0.1; 
    public static final double turnDead = 0.1; 

    public static final double encoderToInchConversion = 1/42 * ((Math.PI * 4) / 8.45);
    
    public static final double inchToRev = (8.45/12.56); 
    public static final double revToInch = (12.56/8.45); 
  
  }

  public static class ShooterConstants{
    public static final int topShooterMotorId = 5; 
    public static final int bottomShooterMotorId = 6; 

    public static final double shooterFullSpeed = 1.0; 
    public static final double shooterOffSpeed = 0.0; 


    // perfect speed at subwoofer 

    // lower should be for top speed 
    // higher number should be for bottom roller 

    // public static final double topSpeed = 0.30; 
    // public static final double bottomSpeed = 1.0;

    public static final double topSpeed = 0.26;
    public static final double bottomSpeed = 0.28;


    public static final double subwooferSpeedTop = 0.50; 
    public static final double subwooferSpeedBottom = 0.50; 
    // public static final double subwooferSpeedTop = 0.20; 
    // public static final double subwooferSpeedBottom = 0.50; 
    
    
    // public static final double topSpeed = 1.0;
    // public static final double bottomSpeed = 1.0;


    // amp 
    // public static final double bottomSpeed = 0.00;
    // public static final double topSpeed = 0.20;
  }

  public static class IntakeConstants{
    public static final int intakeMotorId = 7; 
    public static final double intakeSpeed = 0.40; 
    public static final double intakeVelocity = 0.10 * 11000; 
    public static final double outtakeVelocity = -0.10 * 11000; 
    public static final double intakeStopSpeed = 0; 
    public static final double intakeStopVelocity = 0; 
    public static final double outtakeSpeed = -0.70; 
  }


}

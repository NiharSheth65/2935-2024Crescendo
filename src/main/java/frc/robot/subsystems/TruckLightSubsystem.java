// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.simulation.DigitalPWMDataJNI;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;
import frc.robot.commands.ledCommands.LedCommand;

public class TruckLightSubsystem extends SubsystemBase {
  /** Creates a new TruckLightSubsystem. */

  // private PWM pwmLight; 


  private DigitalOutput LED; 
  public static final int LED_DIO_CHANNEL = LedConstants.truckLightPort;

  public TruckLightSubsystem() {
    LED = new DigitalOutput(LED_DIO_CHANNEL);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
  public void setTruckBrightness(double brightness){
    boolean on = brightness > 0.5; // Example threshold for on/off
    if (on) {
        // Turn on the LED for a portion of the time based on brightness
        // Adjust the delay to control the perceived brightness
        LED.set(true);
    } else {
        // Turn off the LED
        LED.set(false);
    }
  }
}

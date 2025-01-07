// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.OIConstants;

/** The Operating Interface 
 * This class is abstract and has a private constructor to prevent instantiation.
*/
public abstract class OI {
  private OI() {}

  // The Driver's joystick
  public static final Joystick driverJoytick = new Joystick(OIConstants.DRIVER_CONTROLLER_PORT);
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.utils.Utils;

/** The Operating Interface 
 * This class is abstract and has a private constructor to prevent instantiation.
*/
public abstract class OI {
  private OI() {}

  // The Driver's joystick
  public static final CommandXboxController driverController = new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);
  public static final CommandXboxController auxController = new CommandXboxController(OIConstants.AUX_CONTROLLER_PORT);

  public static double[] getMappedJoysticks() {
    // Convert XY to polar for mapping
    double inputTranslationDir = Math.atan2(driverController.getLeftY(), driverController.getLeftX());
    double inputTranslationMag = Math.sqrt(Math.pow(driverController.getLeftY(), 2) + Math.pow(driverController.getLeftX(), 2));

    // Deadband
    if(inputTranslationMag <= OIConstants.DRIVER_DEADBAND) {
      return new double[] {0,0};
    // Exceeding the maximum value
    } else if(inputTranslationMag > 1) {
      return new double[] {0.70710678118, 0.70710678118};
    }

    double outputTranslationMag = mappingFunction(inputTranslationMag);

    // Converting the polar coords back to cartesian coords.
    return new double[] {
        outputTranslationMag * Math.cos(inputTranslationDir),
        outputTranslationMag * Math.sin(inputTranslationDir)
    };
  }

  /**
   * The mapping function is a collection of three,
   * fifth order, polynomial, functions combined together
   * to estimate a cubic bezier curve. To calculate the
   * bezier curve and the following polynomial functions,
   * use the Desmos graph I made below. Make your own graph
   * that fits your driving style the best.
   * @link https://www.desmos.com/calculator/g8ndnhvoxr
   * @param x Input value
   * @return Output value mapped to the function
   */
  public static double mappingFunction(double x) {
    x = Utils.normalize(x);
    
    if(x <= 0.759808) {
      return
          0.452515 * Math.pow(x, 5) +
          -0.619042 * Math.pow(x, 4) +
          0.332171 * Math.pow(x, 3) +
          -0.111379 * Math.pow(x, 2) +
          0.585934 * x + 
          -0.0000825337;
    } else if(x <= 0.935956) {
      return
          1402.36289 * Math.pow(x, 5) +
          -5750.3564 * Math.pow(x, 4) +
          9436.1622 * Math.pow(x, 3) +
          -7743.9163 * Math.pow(x, 2) +
          3178.27035 * x + 
          -521.5418;
    } else if(x <= 1) {
      return
          133388.9857 * Math.pow(x, 5) +
          -658439.6582 * Math.pow(x, 4) +
          1298943.527 * Math.pow(x, 3) +
          -1280124.655 * Math.pow(x, 2) +
          630239.1878 * x + 
          -124006.3873;
    } else {
      return 0;
    }
  }
}

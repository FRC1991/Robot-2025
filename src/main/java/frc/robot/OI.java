// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
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

  public static double getDriveLeftX() {
    return MathUtil.applyDeadband(
      OI.mappingFunction(OI.driverController.getLeftX()),
      OIConstants.DRIVER_DEADBAND
    );
  }

  public static double getDriveLeftY() {
    return MathUtil.applyDeadband(
      OI.mappingFunction(OI.driverController.getLeftY()),
      OIConstants.DRIVER_DEADBAND
    );
  }

  public static double getDriveRightX() {
    return MathUtil.applyDeadband(
      OI.mappingFunction(OI.driverController.getRightX()),
      OIConstants.DRIVER_DEADBAND
    );
  }

  public static double getDriveRightY() {
    return MathUtil.applyDeadband(
      OI.mappingFunction(OI.driverController.getRightY()),
      OIConstants.DRIVER_DEADBAND
    );
  }

  // A half second rumble on the aux controller
  public static void rumbleAuxController() {
    Thread rumble = new Thread(() -> {
      auxController.setRumble(RumbleType.kBothRumble, .9);
      Timer.delay(0.5);
      auxController.setRumble(RumbleType.kBothRumble, 0);
    });
    rumble.start();
  }

  // A half second rumble on the driver controller
  public static void rumbleDriverController() {
    Thread rumble = new Thread(() -> {
      driverController.setRumble(RumbleType.kBothRumble, .9);
      Timer.delay(0.5);
      driverController.setRumble(RumbleType.kBothRumble, 0);
    });
    rumble.start();
  }

  public static void rumbleControllers() {
    rumbleAuxController();
    rumbleDriverController();
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
    
    if(x > 0) {
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
    } else {
      x = Math.abs(x);

      if(x <= 0.759808) {
        return -1 * (
          0.452515 * Math.pow(x, 5) +
          -0.619042 * Math.pow(x, 4) +
          0.332171 * Math.pow(x, 3) +
          -0.111379 * Math.pow(x, 2) +
          0.585934 * x + 
          -0.0000825337);
      } else if(x <= 0.935956) {
        return -1 * (
          1402.36289 * Math.pow(x, 5) +
          -5750.3564 * Math.pow(x, 4) +
          9436.1622 * Math.pow(x, 3) +
          -7743.9163 * Math.pow(x, 2) +
          3178.27035 * x + 
          -521.5418);
      } else if(x <= 1) {
        return -1 * (
          133388.9857 * Math.pow(x, 5) +
          -658439.6582 * Math.pow(x, 4) +
          1298943.527 * Math.pow(x, 3) +
          -1280124.655 * Math.pow(x, 2) +
          630239.1878 * x + 
          -124006.3873);
      } else {
        return 0;
      }
    }
  }
}

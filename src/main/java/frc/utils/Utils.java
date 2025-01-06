package frc.utils;

import frc.robot.Constants;

public class Utils {
  private Utils() {}

  /**
   * All speeds that a motor is set to should be between -1 and 1.
   * This method ensures that all values will be safe for a motor to run at.
   * @param input The desired speed of a motor
   * @return The desired speed normalized between [-1, 1]
   */
  public static double normalize(double input) {
    if(input > 1) {
      return 1;
    } else if(input < -1) {
      return -1;
    } else if(Double.isNaN(input) || Double.isInfinite(input)) {
      return 0;
    }
    return input;
  }

  /**
   *
   * @param ty Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
   * @return The distance from the Limelight to the Apriltag
   */
  public static double getDistanceToTag(double ty) {
    return (Constants.APRILTAG_HEIGHT - Constants.LIMELIGHT_HEIGHT) / Math.tan(ty);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** 
 * Holds all constants for the robot.
 * 
 * This class is abstract to prevent instantiation. All constants should have
 * the public, static, and final, modifiers and follow a consistent naming
 * structure.
 */
public abstract class Constants {
  public static final double LIMELIGHT_HEIGHT = 0;
  public static final double APRILTAG_HEIGHT = 0;

  public static abstract class SwerveConstants {
    // Chassis configuration
    public static final double TRACK_WIDTH = Units.inchesToMeters(99);
    // Distance between centers of right and left wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(99);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
    public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
    public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
    public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;
  }

  public static abstract class ModuleConstants {
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
    // This is the L1 gear reduction on the MK4 modules 
    public static final double DRIVING_MOTOR_REDUCTION = 8.14;
    public static final double DRIVING_MOTOR_FREE_SPEED_RPM = 5800;
    public static final double DRIVING_VELOCITY_FEED_FORWARD = 1 / ModuleConstants.DRIVING_MOTOR_FREE_SPEED_RPM;
  }

  // CAN IDs for every CAN device on the robot
  public static abstract class CANConstants {
    // Swerve driving motors
    public static final int FRONT_LEFT_DRIVING_CANID = 2;
    public static final int FRONT_RIGHT_DRIVING_CANID = 3;
    public static final int BACK_LEFT_DRIVING_CANID = 4;
    public static final int BACK_RIGHT_DRIVING_CANID = 5;

    // Swerve turning motors
    public static final int FRONT_LEFT_TURNING_CANID = 6;
    public static final int FRONT_RIGHT_TURNING_CANID = 7;
    public static final int BACK_LEFT_TURNING_CANID = 8;
    public static final int BACK_RIGHT_TURNING_CANID = 9;

    // Gyro
    public static final int GYRO_CANID = 10;
  }
}

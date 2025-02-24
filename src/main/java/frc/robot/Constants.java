// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/** 
 * Holds all constants for the robot.
 * 
 * This class is abstract and has a private constructor to prevent instantiation.
 * All constants should have the public, static, and final,
 * modifiers and follow a consistent naming structure.
 */
public abstract class Constants {
  private Constants() {}
  
  public static final double LIMELIGHT_HEIGHT = 0;
  public static final double APRILTAG_HEIGHT = 0;
  public static final int NEO_CURRENT_LIMIT = 40;

  public static abstract class SwerveConstants {
    public static final RobotConfig PP_CONFIG = new RobotConfig(
        50,
        11.657,
        new ModuleConfig(
            0.102,
            5.45,
            1.2,
            DCMotor.getKrakenX60(1),
            80,
            1),
        new Translation2d(0.4,0.4),
        new Translation2d(0.4,-0.4),
        new Translation2d(-0.4,0.4),
        new Translation2d(-0.4,-0.4)
    );

    // The speed modifier for the swerve drive as a percent
    public static final double SPEED_SCALE = 1;

    // Chassis configuration
    public static final double TRACK_WIDTH = Units.inchesToMeters(22.5);
    // Distance between centers of right and left wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(22.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -(Math.PI / 2) - (Math.PI / 6) - 0.226;
    public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI + 0.226;
    public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = 0 - 0.226; // GOOD
    public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = -(Math.PI / 2) + 0.226;

    public static final boolean GYRO_REVERSED = true;

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double MAX_SPEED_METERS_PER_SECOND = 5.5;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 4 * Math.PI; // radians per second
    public static final double MAX_DEGREES_PER_SCHEDULER_LOOP = MAX_ANGULAR_SPEED_RADIANS_PER_SECOND * (180/Math.PI) / 1000 * 20 * 0.7;
  }

  public static abstract class ModuleConstants {
    // The standard size for Billet wheels which come standard with the MK4 modules
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
    // This is the L1 gear reduction on the MK4 modules 
    public static final double DRIVING_MOTOR_REDUCTION = 8.14;
    public static final double TURNING_MOTOR_REDUCTION = 12.8;
    // The driving motor is a Krakenx60 without FOC
    public static final double DRIVING_MOTOR_FREE_SPEED_RPM = 5800;
    public static final double DRIVING_VELOCITY_FEED_FORWARD = 1 / ModuleConstants.DRIVING_MOTOR_FREE_SPEED_RPM;
  }

  // CAN IDs for every CAN device on the robot
  public static abstract class CANConstants {
    // Swerve driving motors
    public static final int FRONT_LEFT_DRIVING_ID = 6;
    public static final int FRONT_RIGHT_DRIVING_ID = 7;
    public static final int BACK_LEFT_DRIVING_ID = 9;
    public static final int BACK_RIGHT_DRIVING_ID = 8;

    // Swerve turning motors
    public static final int FRONT_LEFT_TURNING_ID = 2;
    public static final int FRONT_RIGHT_TURNING_ID = 3;
    public static final int BACK_LEFT_TURNING_ID = 5;
    public static final int BACK_RIGHT_TURNING_ID = 4;

    public static final int FL_ENCODER_ANALOG_INPUT_CHANNEL = 1;
    public static final int FR_ENCODER_ANALOG_INPUT_CHANNEL = 2;
    public static final int BL_ENCODER_ANALOG_INPUT_CHANNEL = 0;
    public static final int BR_ENCODER_ANALOG_INPUT_CHANNEL = 3;

    public static final int PROXIMITY_SENSOR_CHANNEL = 0;

    // Gyro
    public static final int GYRO_ID = 10;

    public static final int SPITTER_MOTOR_ONE_ID = 9990;
    public static final int SPITTER_MOTOR_TWO_ID = 9991;
    public static final int PIVOT_ID = 9992;
    public static final int ALGAE_INTAKE_ID = 9993;
    public static final int ELEVATOR_MOTOR_ONE_ID = 9994;
    public static final int ELEVATOR_MOTOR_TWO_ID = 9995;
    public static final int FLIPPER_ID = 9996;
  }

  public static abstract class OIConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final double DRIVER_DEADBAND = 0.05;
  }

  public static abstract class PivotConstants {
    // These are arbitrary values
    public static final double STORED_POSITION = 0;
    public static final double SCORING_POSITION = 5;
    public static final double INTAKE_POSITION = 8;
  }

  public static abstract class SpitterConstants {
    public static final double MOTOR_SPEED = 1;
  }

  public static abstract class AlgaeConstants {
    public static final double INTAKE_SPEED = 1;
    public static final double SCORING_SPEED = -1;
  }

  public static abstract class ElevatorConstants {
    public static final double ELEVATOR_MOTOR_REDUCTION = 1;
    public static final double L1_POSITION_INCHES = 19;
    public static final double L2_POSITION_INCHES = 32;
    public static final double STORED_POSITION = 0;
    public static final double INTAKING_POSITION = 0;
  }
}

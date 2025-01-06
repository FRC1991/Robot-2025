// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.NoSuchElementException;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.TeleopConstants;
import frc.robot.OperatingInterface;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase implements CheckableSubsystem, StateSubsystem {
  // Create SwerveModules
  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  private boolean status = false;
  private boolean initialized = false;

  // The gyro sensor
  public final Pigeon2 m_gyro = new Pigeon2(DriveConstants.kGyroId);

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;
  //private double speedScale;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(getHeading()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  private ChassisSpeeds m_RobotChassisSpeeds = new ChassisSpeeds();

  private static Swerve m_Instance;

  private SwerveStates desiredState, currentState = SwerveStates.IDLE;

  private PIDController angleController = new PIDController(0.009, 0, 0);

  private double sourceAngle = -1;

  private DoubleSupplier aimingAngle;

  // Constructor is private to prevent multiple instances from being made
  private Swerve() {
    angleController.setTolerance(1);
    angleController.enableContinuousInput(0, 360);

    zeroHeading();

    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(0.009, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    //TODO calculate real radius, with bumpers on
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

    // Continuously checks for alliance until correct angle is chosen
    try {
      if(DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
        sourceAngle = 145;
      } else {
        // RED_SIDE_ANGLE = ((180 - BLUE_SIDE_ANGLE) + 180)
        sourceAngle = 215;
      }
      initialized = true;
    } catch(NoSuchElementException e) {
      /*
       * I like blue alliance better, so I'm setting the angle to blue
       * if we aren't connected to the driver station.
       * 75% of people also like blue alliance best according to our
       * silly goofy pit scouting at BattleCry 2024
       */
      sourceAngle = 145;
    }
  }

  /**
   * @return The main Pivot object
   */
  public static Swerve getInstance() {
    if(m_Instance == null) {
      m_Instance = new Swerve();
    }
    return m_Instance;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(getHeading()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  /**
   * @param getter A method to get the offset from the target in degrees
  */
  public void setAngleSupplier(DoubleSupplier getter) {
    aimingAngle = getter;
  }

  /**
   * @return True, if the robot heading is within one degree of the target
   * <li> False, if the heading is not withing the tolerance.
   */
  public boolean facingTarget() {
    return angleController.atSetpoint();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(getHeading()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Resets the odometry to 0 degrees and 0 velocity
   */
  public void resetOdometry() {
    resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
  }

  /**
   * Used in the Pathplanner AutoBuilder.configureHolonomic
   *
   * @return The ChassisSpeeds relative to the robot
   */
  private ChassisSpeeds getRobotRelativeSpeeds() {
    return m_RobotChassisSpeeds;
  }

  /**
   * Drives the robot according to ChassisSpeeds provided
   * by Pathplanner during auto
   *
   * @param t ChassisSpeeds relative to the robot
   */
  private void drive(ChassisSpeeds t) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(t);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }


      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain and scaling the speed
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    m_RobotChassisSpeeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(getHeading()))
            : m_RobotChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   * @param speedScale    A percent to shrink the robot speed by. Must be between 0-1.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit, double speedScale) {
    xSpeed *= speedScale;
    ySpeed *= speedScale;
    rot *= speedScale + ((1 - speedScale) / 2);

    drive(xSpeed, ySpeed, rot, fieldRelative, rateLimit);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the wheels into an || formation to prevent movement.
   */
  public void setTank() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(180)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(1800)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.setYaw(0);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from [0, 360)
   */
  public double getHeading() {
    return Units.radiansToDegrees(SwerveUtils.WrapAngle(Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()).getRadians()));
  }

  /**
   * Returns the heading of the robot relative to a target angle
   *
   * @return the robot's heading in degrees (180, -180), optimized for the TurnToAngle PID command
   */
  public double getHeadingTurnToAngle(double target) {
    double angle = Units.radiansToDegrees(SwerveUtils.WrapAngle(Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()).getRadians()));

    if(angle > (target + 180)) {
      angle -= 360;
    } else if(angle < (target - 180)) {
      angle += 360;
    }

    return angle;
  }
  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Stops movement in all motors
   */
  @Override
  public void stop() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_rearLeft.stop();
    m_rearRight.stop();
  }

  /**
   * @return Is the subsystem is okay to operate
   */
  @Override
  public boolean checkSubsystem() {
    status = m_frontLeft.checkSubsystem();
    status &= m_frontRight.checkSubsystem();
    status &= m_rearLeft.checkSubsystem();
    status &= m_rearRight.checkSubsystem();
    status &= getInitialized();
    status &= currentState != SwerveStates.BROKEN;

    return status;
  }

  /**
   * @return Has the constructor been executed
   */
  @Override
  public boolean getInitialized() {
    return initialized && aimingAngle != null;
  }

  /**
   * Updates any information the subsystem needs
   */
  @Override
  public void update() {
    switch(currentState) {
      case IDLE:
        break;
      case BROKEN:
        break;
      case DRIVE:
        if(DriverStation.isTeleopEnabled()) {
          drive(
            -MathUtil.applyDeadband(OperatingInterface.driverJoytick.getRawAxis(1), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(OperatingInterface.driverJoytick.getRawAxis(0), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(OperatingInterface.driverJoytick.getRawAxis(2), OIConstants.kDriveDeadband),
            true, false, TeleopConstants.kSwerveSpeed);
        }
        break;
      case PICKUP:
        if(DriverStation.isTeleopEnabled()) {
          drive(
              -MathUtil.applyDeadband(OperatingInterface.driverJoytick.getRawAxis(1), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(OperatingInterface.driverJoytick.getRawAxis(0), OIConstants.kDriveDeadband),
              angleController.calculate(getHeading()),
              true, false, TeleopConstants.kSwerveSpeed * 0.6);
        }
        break;
      case AIMING:
        if(DriverStation.isTeleopEnabled()) {
          drive(
              -MathUtil.applyDeadband(OperatingInterface.driverJoytick.getRawAxis(1), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(OperatingInterface.driverJoytick.getRawAxis(0), OIConstants.kDriveDeadband),
              angleController.calculate(aimingAngle.getAsDouble()),
              true, false, TeleopConstants.kSwerveSpeed);
        }
        break;
      case LOCKED:
        break;

      default:
        break;
    }

    if(!checkSubsystem()) {
      setDesiredState(SwerveStates.BROKEN);
    }
  }

  /**
   * Handles moving from one state to another
   */
  @Override
  public void handleStateTransition() {
    switch(desiredState) {
      case IDLE:
        drive(0,0,0,false,false);
        break;
      case BROKEN:
        stop();
        break;
      case DRIVE:
        break;
      case PICKUP:
        angleController.setSetpoint(sourceAngle);
        break;
      case AIMING:
        angleController.setSetpoint(0);
        break;
      case LOCKED:
        setX();
        break;

      default:
        break;
    }

    currentState = desiredState;
  }

  /**
   * Sets the desired state of the subsystem
   * @param state Desired state
   */
  public void setDesiredState(SwerveStates state) {
    if(this.desiredState != state /*&& this.currentState != SwerveStates.BROKEN*/) {
      desiredState = state;
      handleStateTransition();
    }
  }

  /**
   * @return The current state of the subsystem
   */
  public SwerveStates getState() {
    return currentState;
  }

  public enum SwerveStates {
    IDLE,
    BROKEN,
    /** Regular control of the robot */
    DRIVE,
    /** Slows the robot to sixty percent of its normal speed and uses the angle PID controller
     * to face perpendicular to the Source. This takes away yaw control.
    */
    PICKUP,
    /** Uses the angle PID controller minimize the offset provided by aimingAngle.
     * This takes away yaw control
     */
    AIMING,
    /** Removes all control and locks the wheels in an X formation */
    LOCKED;
  }
}

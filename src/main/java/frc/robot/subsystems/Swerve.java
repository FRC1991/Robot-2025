// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.OI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase implements CheckableSubsystem, StateSubsystem {
  // Create SwerveModules
  private final SwerveModule m_frontLeft = new SwerveModule(
      CANConstants.FRONT_LEFT_DRIVING_ID,
      CANConstants.FRONT_LEFT_TURNING_ID,
      CANConstants.FL_ENCODER_ANALOG_INPUT_CHANNEL,
      SwerveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

  private final SwerveModule m_frontRight = new SwerveModule(
      CANConstants.FRONT_RIGHT_DRIVING_ID,
      CANConstants.FRONT_RIGHT_TURNING_ID,
      CANConstants.FR_ENCODER_ANALOG_INPUT_CHANNEL,
      SwerveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

  private final SwerveModule m_rearLeft = new SwerveModule(
      CANConstants.BACK_LEFT_DRIVING_ID,
      CANConstants.BACK_LEFT_TURNING_ID,
      CANConstants.BL_ENCODER_ANALOG_INPUT_CHANNEL,
      SwerveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET);

  private final SwerveModule m_rearRight = new SwerveModule(
      CANConstants.BACK_RIGHT_DRIVING_ID,
      CANConstants.BACK_RIGHT_TURNING_ID,
      CANConstants.BR_ENCODER_ANALOG_INPUT_CHANNEL,
      SwerveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);

  private boolean status = false;
  private boolean initialized = false;

  // The gyro sensor
  public final Pigeon2 m_gyro = new Pigeon2(CANConstants.GYRO_ID);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      SwerveConstants.DRIVE_KINEMATICS,
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

  // private double desiredHeading;
  private PIDController angleController = new PIDController(0.009, 0, 0);

  private DoubleSupplier aimingAngle;

  // Constructor is private to prevent multiple instances from being made
  private Swerve() {

    Shuffleboard.getTab("Main").add("Gyro", m_gyro.getYaw().getValueAsDouble());

    angleController.setTolerance(1);
    angleController.enableContinuousInput(0, 360);

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> drive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        ),
        SwerveConstants.PP_CONFIG, // The robot configuration
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

    zeroHeading();
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
    var swerveModuleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(t);
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
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain and scaling the speed
    double xSpeedDelivered = xSpeed * SwerveConstants.MAX_SPEED_METERS_PER_SECOND;
    double ySpeedDelivered = ySpeed * SwerveConstants.MAX_SPEED_METERS_PER_SECOND;
    double rotDelivered = rot * SwerveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

    var swerveModuleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(getHeading()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    
    setModuleStates(swerveModuleStates);
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
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double speedScale) {
    xSpeed *= speedScale;
    ySpeed *= speedScale;
    rot *= speedScale + ((1 - speedScale) / 2);

    drive(xSpeed, ySpeed, rot, fieldRelative);
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
   * Sets the wheels to their zero.
   */
  public void setTank() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, SwerveConstants.MAX_SPEED_METERS_PER_SECOND);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Gets the current module states
   * 
   * @return Swerve module states of the robot
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    };
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetDriveEncoders() {
    m_frontLeft.resetDriveEncoder();
    m_rearLeft.resetDriveEncoder();
    m_frontRight.resetDriveEncoder();
    m_rearRight.resetDriveEncoder();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.setYaw(0);
  }

  // public void setDesiredHeading(double heading) {
  //   desiredHeading = heading;
  // }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from [0, 360)
   */
  public double getHeading() {
    return MathUtil.inputModulus(m_gyro.getYaw().getValueAsDouble(), 0, 360);
  }

  /**
   * Returns the heading of the robot relative to a target angle
   *
   * @return the robot's heading in degrees (180, -180), optimized for the TurnToAngle PID command
   */
  public double getHeadingAroundAngle(double target) {
    return MathUtil.inputModulus(target - m_gyro.getRotation2d().getDegrees(), -180, 180);
  }
  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getAngularVelocityZWorld().getValueAsDouble() * (SwerveConstants.GYRO_REVERSED ? -1.0 : 1.0);
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
              MathUtil.applyDeadband(OI.driverJoytick.getY(), 0.05),
              MathUtil.applyDeadband(OI.driverJoytick.getX(), 0.05),
              MathUtil.applyDeadband(OI.driverJoytick.getZ(), 0.05),
              true, SwerveConstants.SPEED_SCALE);
        }
        break;
      case AIMING:
        if(DriverStation.isTeleopEnabled()) {
          drive(
              -OI.getMappedJoysticks()[0],
              -OI.getMappedJoysticks()[1],
              -OI.mappingFunction(aimingAngle.getAsDouble()),
              true, SwerveConstants.SPEED_SCALE);
        }
        break;
      case LOCKED:
        setX();
        break;

      default:
        break;
    }

    // if(!checkSubsystem()) {
    //   setDesiredState(SwerveStates.BROKEN);
    // }
  }

  /**
   * Handles moving from one state to another
   */
  @Override
  public void handleStateTransition() {
    switch(desiredState) {
      case IDLE:
        stop();
        break;
      case BROKEN:
        stop();
        break;
      case DRIVE:
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
    if(this.desiredState != state) {
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

  /**
   * The list of possible states for this subsystem
   */
  public enum SwerveStates {
    IDLE,
    BROKEN,
    /** Regular control of the robot */
    DRIVE,
    /** Uses the angle PID controller minimize the offset provided by aimingAngle.
     * This takes away yaw control from the driver.
     */
    AIMING,
    /** Removes all control and locks the wheels in an X formation */
    LOCKED;
  }
}

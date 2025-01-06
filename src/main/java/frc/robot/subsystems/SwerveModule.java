// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;

import frc.robot.Constants.ModuleConstants;

public class SwerveModule implements CheckableSubsystem {
  private boolean status = false;
  private boolean initialized = false;

  private final TalonFX driveMotor;
  private final SparkMax turningMotor;

  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a SwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   *
   * I am NOT using getInstance() here because we intend to have multiple
   * instances of this class.
   */
  public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    driveMotor = new TalonFX(drivingCANId);
    turningMotor = new SparkMax(turningCANId, MotorType.kBrushless);

    m_turningEncoder = turningMotor.getAbsoluteEncoder();

    m_turningClosedLoopController = turningMotor.getClosedLoopController();

    TalonFXConfiguration driveTalonConfig = new TalonFXConfiguration();
    SparkMaxConfig turningConfig = new SparkMaxConfig();

    // Use module constants to calculate conversion factors and feed forward gain.
    double drivingFactor = ModuleConstants.WHEEL_DIAMETER_METERS * Math.PI / ModuleConstants.DRIVING_MOTOR_REDUCTION;
    double turningFactor = 2 * Math.PI;

    driveTalonConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    driveTalonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    driveTalonConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
    driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveTalonConfig.Feedback.SensorToMechanismRatio = drivingFactor;

    driveTalonConfig.Slot0.kP = 0.04;
    driveTalonConfig.Slot0.kI = 0;
    driveTalonConfig.Slot0.kD = 0;

    driveMotor.getConfigurator().apply(driveTalonConfig, 0.1);
    driveMotor.optimizeBusUtilization(0, 0.1);

    turningConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20);
    turningConfig.absoluteEncoder
        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of the steering motor in the MAXSwerve Module.
        .inverted(true)
        .positionConversionFactor(turningFactor) // radians
        .velocityConversionFactor(turningFactor / 60.0); // radians per second
    turningConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        // These are example gains you may need to them for your own robot!
        .pid(1, 0, 0)
        .outputRange(-1, 1)
        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, turningFactor);

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    driveMotor.setPosition(0);

    initialized = true;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    driveMotor.getVelocity().refresh();
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(driveMotor.getVelocity().getValueAsDouble(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    driveMotor.getPosition().refresh();
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        driveMotor.getPosition().getValueAsDouble(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning motors towards their respective setpoints.
    driveMotor.setControl(new VelocityDutyCycle(correctedDesiredState.speedMetersPerSecond).withFeedForward(ModuleConstants.DRIVING_VELOCITY_FEED_FORWARD));
    m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes the drive motor encoder. */
  public void resetDriveEncoder() {
    driveMotor.setPosition(0);
  }

  /**
   * Stops movement in all motors
   */
  @Override
  public void stop() {
    driveMotor.stopMotor();
    turningMotor.stopMotor();
  }

  /**
   * @return Is the subsystem is okay to operate
   */
  @Override
  public boolean checkSubsystem() {
    // status = Utils.checkMotor(driveMotor, driveMotor.getDeviceId());
    // status &= Utils.checkMotor(turningMotor, turningMotor.getDeviceId());
    status &= getInitialized();

    return status;
  }

  /**
   * @return Has the constructor been executed
   */
  @Override
  public boolean getInitialized() {
    return initialized;
  }
}
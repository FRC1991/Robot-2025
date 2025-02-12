// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SwerveConstants;
import frc.utils.Utils;

public class SwerveModule implements CheckableSubsystem {
  private boolean status = false;
  private boolean initialized = false;

  private final TalonFX driveMotor;
  private final SparkMax turningMotor;

  private final AnalogEncoder m_turningEncoder;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  private PIDController azimuth;
  /**
   * Constructs a SwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This robot uses the MK4 swerve modules
   * with a Krakenx60 as the drive motor and a Neo as the azimuth (turning) motor.
   *
   * Do NOT use getInstance() because we intend to have multiple
   * instances of this class.
   */
  public SwerveModule(int drivingCANId, int turningCANId, int encoderChannel, double chassisAngularOffset) {
    driveMotor = new TalonFX(drivingCANId);
    turningMotor = new SparkMax(turningCANId, MotorType.kBrushless);

    azimuth = new PIDController(0.008, 0, 0.00008);
    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 170 degrees
    // to -10 degrees will go through 180 rather than the other direction which is a
    // longer route.
    azimuth.enableContinuousInput(-180, 180);
    azimuth.setTolerance(1);

    m_turningEncoder = new AnalogEncoder(encoderChannel);

    TalonFXConfiguration driveTalonConfig = new TalonFXConfiguration();
    SparkMaxConfig turningConfig = new SparkMaxConfig();

    // Use module constants to calculate conversion factors and feed forward gain.
    double drivingFactor = ModuleConstants.WHEEL_DIAMETER_METERS * Math.PI / ModuleConstants.DRIVING_MOTOR_REDUCTION;

    driveTalonConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    driveTalonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    driveTalonConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
    driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveTalonConfig.Feedback.SensorToMechanismRatio = drivingFactor;

    driveTalonConfig.Slot0.kP = 0.05;
    driveTalonConfig.Slot0.kI = 0;
    driveTalonConfig.Slot0.kD = 0;

    turningConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .encoder.positionConversionFactor(ModuleConstants.TURNING_MOTOR_REDUCTION);

    // Apply the configuration to the motor, so it is always in
    // a consistent state regardless of what has happened to the
    // hardware (being replaced, factory reset, new firmware, etc)
    driveMotor.getConfigurator().apply(driveTalonConfig, 0.1);
    driveMotor.optimizeBusUtilization(0, 0.1);
    driveMotor.getPosition().setUpdateFrequency(4);

    turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    turningMotor.getEncoder().setPosition(m_turningEncoder.get());

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(getEncoderRadians());
    driveMotor.setPosition(0);

    initialized = true;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(driveMotor.getVelocity().getValueAsDouble(),
        new Rotation2d(getEncoderRadians()));
  }

  /**
   * Returns angle in degrees [-180, 180]
   * @return Current angle of the module
   */
  public double getEncoderDegrees() {
    return getEncoderRadians() * (180 / Math.PI);
  }

  /**
   * Returns angle in radians [-PI, PI]
   * @return Current angle of the module
   */
  public double getEncoderRadians() {
    return MathUtil.angleModulus((m_turningEncoder.get() * 2 * Math.PI) + m_chassisAngularOffset);
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // driveMotor.getPosition().refresh();
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        driveMotor.getPosition().getValueAsDouble(),
        new Rotation2d(getEncoderRadians() - m_chassisAngularOffset));
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
    correctedDesiredState.angle = desiredState.angle;

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(getEncoderRadians()));

    // Command driving and turning motors towards their respective setpoints.
    driveMotor.set(Utils.normalize(correctedDesiredState.speedMetersPerSecond / SwerveConstants.MAX_SPEED_METERS_PER_SECOND));
    
    // azimuth.setSetpoint(correctedDesiredState.angle.getDegrees());

    double error = azimuth.calculate(getEncoderDegrees(), correctedDesiredState.angle.getDegrees());

    if(Math.abs(error) < 0.02) {
      error = 0;
    }

    turningMotor.set(Utils.normalize(error));

    m_desiredState = correctedDesiredState;
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

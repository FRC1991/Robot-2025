// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.SpitterConstants;

// This is for the coral
public class Spitter extends SubsystemBase implements CheckableSubsystem, StateSubsystem {

  private boolean status = false;
  private boolean initialized = false;

  private SparkMax motor1, motor2;

  private DigitalInput proximitySensor;

  private static Spitter m_Instance;

  private SpitterStates desiredState, currentState = SpitterStates.IDLE;

  /** Creates a new Spitters. */
  private Spitter() {
    motor1 = new SparkMax(CANConstants.SPITTER_MOTOR_ONE_ID, MotorType.kBrushless);
    motor2 = new SparkMax(CANConstants.SPITTER_MOTOR_TWO_ID, MotorType.kBrushless);

    SparkMaxConfig spitterConfig = new SparkMaxConfig();

    spitterConfig.idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.NEO_CURRENT_LIMIT);

    motor1.configure(spitterConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    motor2.configure(spitterConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    proximitySensor = new DigitalInput(CANConstants.PROXIMITY_SENSOR_CHANNEL);

    initialized = true;
    status = true;
  }

  /**
   * @return The main Spitter object
   */
  public static Spitter getInstance() {
    if(m_Instance == null) {
      m_Instance = new Spitter();
    }
    return m_Instance;
  }

  @Override
  public void stop() {
    motor1.stopMotor();
    motor2.stopMotor();
  }

  @Override
  public boolean getInitialized() {
    return initialized;
  }

  @Override
  public boolean checkSubsystem() {
    status = getInitialized();

    return status;
  }

  public void setDesiredState(SpitterStates state) {
    if(this.desiredState != state) {
      desiredState = state;
      handleStateTransition();
    }
  }

  @Override
  public void handleStateTransition() {
    switch(desiredState) {
      case IDLE:
        stop();
        break;
      case BROKEN:
        stop();
        break;
      case RUNNING:
        motor1.set(SpitterConstants.MOTOR_SPEED);
        motor2.set(-SpitterConstants.MOTOR_SPEED);
        break;

      default:
        break;
    }

    currentState = desiredState;
  }

  public void update() {
    switch(currentState) {
      case IDLE:
        break;
      case BROKEN:
        break;
      case RUNNING:
        if(proximitySensor.get()) {
          setDesiredState(SpitterStates.IDLE);
        }
        break;

      default:
        break;
    }

    if(!checkSubsystem()) {
      setDesiredState(SpitterStates.BROKEN);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * @return The current state of the subsystem
   */
  public SpitterStates getState() {
    return currentState;
  }

  public enum SpitterStates {
    IDLE,
    BROKEN,
    RUNNING;
  }
}

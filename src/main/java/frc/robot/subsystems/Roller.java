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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants;
import frc.robot.Constants.RollerConstants;

public class Roller extends SubsystemBase implements CheckableSubsystem, StateSubsystem {

  private boolean status = false;
  private boolean initialized = false;

  private SparkMax motor;

  private static Roller m_Instance;

  private RollerStates desiredState, currentState = RollerStates.IDLE;

  /** Creates a new Roller. */
  private Roller() {
    motor = new SparkMax(CANConstants.ROLLER_ID, MotorType.kBrushless);

    SparkMaxConfig rollersConfig = new SparkMaxConfig();

    rollersConfig.idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.NEO_CURRENT_LIMIT);

    motor.configure(rollersConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    initialized = true;
    status = true;
  }

  /**
   * @return The main Elevator object
   */
  public static Roller getInstance() {
    if(m_Instance == null) {
      m_Instance = new Roller();
    }
    return m_Instance;
  }

  @Override
  public void stop() {
    motor.stopMotor();
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

  public void setDesiredState(State state) {
    if(this.desiredState != state) {
      desiredState = (RollerStates) state;
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
      case INTAKING:
        motor.set(RollerConstants.INTAKE_SPEED);
        break;
      case SCORING:
        motor.set(RollerConstants.SCORE_SPEED);
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
      case INTAKING:
        break;
      case SCORING:
        break;

      default:
        break;
    }

    if(!checkSubsystem()) {
      setDesiredState(RollerStates.BROKEN);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * @return The current state of the subsystem
   */
  public RollerStates getState() {
    return currentState;
  }

  public enum RollerStates implements State {
    IDLE,
    BROKEN,
    INTAKING,
    SCORING;
  }
}

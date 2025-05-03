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
import frc.robot.Constants.AlgaeConstants;

public class AlgaeIntake extends SubsystemBase implements CheckableSubsystem, StateSubsystem {

  private boolean status = false;
  private boolean initialized = false;

  private SparkMax motor;

  private static AlgaeIntake m_Instance;

  private AlgaeStates desiredState, currentState = AlgaeStates.IDLE;

  /** Creates a new Algaes. */
  private AlgaeIntake() {
    motor = new SparkMax(CANConstants.ALGAE_INTAKE_ID, MotorType.kBrushless);

    SparkMaxConfig algaeIntakeConfig = new SparkMaxConfig();

    algaeIntakeConfig.idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.CURRENT_LIMIT_550);

    motor.configure(algaeIntakeConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    initialized = true;
    status = true;
  }

  /**
   * @return The main Elevator object
   */
  public static AlgaeIntake getInstance() {
    if(m_Instance == null) {
      m_Instance = new AlgaeIntake();
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
      desiredState = (AlgaeStates) state;
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
        motor.set(AlgaeConstants.INTAKE_SPEED);
        break;
      case SCORING:
        motor.set(AlgaeConstants.SCORING_SPEED);
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
      setDesiredState(AlgaeStates.BROKEN);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * @return The current state of the subsystem
   */
  public AlgaeStates getState() {
    return currentState;
  }

  public enum AlgaeStates implements State {
    IDLE,
    BROKEN,
    INTAKING,
    SCORING;
  }
}

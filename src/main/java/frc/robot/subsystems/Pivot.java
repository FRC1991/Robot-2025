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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.PivotConstants;
import frc.utils.Utils;
import frc.utils.Utils.ElasticUtil;

// This is for the coral
public class Pivot extends SubsystemBase implements CheckableSubsystem, StateSubsystem {

  private boolean status = false;
  private boolean initialized = false;

  private SparkMax motor;

  private PIDController posController;

  private static Pivot m_Instance;

  private double p = 0.01, i = 0, d = 0;

  private PivotStates desiredState, currentState = PivotStates.IDLE;

  /** Creates a new Pivots. */
  private Pivot() {
    motor = new SparkMax(CANConstants.PIVOT_ID, MotorType.kBrushless);

    SparkMaxConfig pivotConfig = new SparkMaxConfig();

    pivotConfig.idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.NEO_CURRENT_LIMIT);

    motor.configure(pivotConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    ElasticUtil.putDouble("pivot P", () -> this.p, value -> {this.p=value;});
    ElasticUtil.putDouble("pivot I", () -> this.i, value -> {this.i=value;});
    ElasticUtil.putDouble("pivot D", () -> this.d, value -> {this.d=value;});

    posController = new PIDController(p, i, d);
    posController.setTolerance(PivotConstants.PID_ERROR_TOLERANCE);

    initialized = true;
    status = true;
  }

  /**
   * @return The main Pivot object
   */
  public static Pivot getInstance() {
    if(m_Instance == null) {
      m_Instance = new Pivot();
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

  public void setDesiredState(PivotStates state) {
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
      case STORED:
        posController.setSetpoint(PivotConstants.STORED_POSITION);
        break;
      case SCORING:
        posController.setSetpoint(PivotConstants.SCORING_POSITION);
        break;
      case INTAKING:
        posController.setSetpoint(PivotConstants.INTAKE_POSITION);
        break;

      default:
        break;
    }

    currentState = desiredState;
  }

  public void update() {
    posController.setPID(p, i, d);

    switch(currentState) {
      case IDLE:
        break;
      case BROKEN:
        break;
      case STORED:
      case SCORING:
      case INTAKING:
        motor.set(Utils.normalize(posController.calculate(motor.getEncoder().getPosition())));
        break;

      default:
        break;
    }

    if(!checkSubsystem()) {
      setDesiredState(PivotStates.BROKEN);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * @return The current state of the subsystem
   */
  public PivotStates getState() {
    return currentState;
  }

  public enum PivotStates {
    IDLE,
    BROKEN,
    STORED,
    SCORING,
    INTAKING;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.utils.Utils;
import frc.utils.Utils.ElasticUtil;

public class Elevator extends SubsystemBase implements CheckableSubsystem, StateSubsystem {

  private boolean status = false;
  private boolean initialized = false;

  private SparkMax motor1, motor2;

  private PIDController posController;

  private static Elevator m_Instance;

  private ElevatorStates desiredState, currentState = ElevatorStates.IDLE;

  /** Creates a new Arm. */
  private Elevator() {
    motor1 = new SparkMax(CANConstants.ELEVATOR_MOTOR_ONE_ID, MotorType.kBrushless);
    motor2 = new SparkMax(CANConstants.ELEVATOR_MOTOR_TWO_ID, MotorType.kBrushless);

    SparkMaxConfig elevatorConfig = new SparkMaxConfig();

    elevatorConfig.idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.NEO_CURRENT_LIMIT)
        .encoder.positionConversionFactor(ElevatorConstants.ELEVATOR_MOTOR_REDUCTION);

    motor1.configure(elevatorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    motor2.configure(elevatorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    posController = new PIDController(2, 0, 0);
    posController.setTolerance(ElevatorConstants.PID_ERROR_TOLERANCE);

    motor1.getEncoder().setPosition(0);
    motor2.getEncoder().setPosition(0);

    ElasticUtil.putDouble("Elevator Position", this::getEncoder);
    ElasticUtil.putDouble("elevator pos 1", motor1.getEncoder()::getPosition);
    ElasticUtil.putDouble("elevator pos 2", motor2.getEncoder()::getPosition);
    ElasticUtil.putBoolean("elevator at setpoint", this::atSetpoint);

    initialized = true;
    status = true;
  }

  /**
   * @return The main Elevator object
   */
  public static Elevator getInstance() {
    if(m_Instance == null) {
      m_Instance = new Elevator();
    }
    return m_Instance;
  }

  private void set(double speed) {
    motor1.set(speed);
    motor2.set(-speed);
  }

  @Override
  public void stop() {
    motor1.stopMotor();
    motor2.stopMotor();
  }

  public double getEncoder() {
    return (motor2.getEncoder().getPosition() - motor1.getEncoder().getPosition()) / 2;
  }

  public boolean atSetpoint() {
    return posController.atSetpoint();
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

  public void setDesiredState(ElevatorStates state) {
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
      case INTAKING:
        posController.setSetpoint(ElevatorConstants.INTAKING_POSITION);
        break;
      case STORED:
        posController.setSetpoint(ElevatorConstants.STORED_POSITION);
        break;
      case L1:
        posController.setSetpoint(ElevatorConstants.L1_POSITION_INCHES);
        break;
      case L2:
        posController.setSetpoint(ElevatorConstants.L2_POSITION_INCHES);
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
        set(Utils.normalize(-posController.calculate(getEncoder())));
        break;
      case STORED:
        set(Utils.normalize(-posController.calculate(getEncoder())));
        break;
      case L1:
        set(Utils.normalize(-posController.calculate(getEncoder())));
        break;
      case L2:
        set(Utils.normalize(-posController.calculate(getEncoder())));
        break;
        

      default:
        break;
    }

    if(!checkSubsystem()) {
      setDesiredState(ElevatorStates.BROKEN);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * @return The current state of the subsystem
   */
  public ElevatorStates getState() {
    return currentState;
  }

  public enum ElevatorStates {
    IDLE,
    BROKEN,
    INTAKING,
    STORED,
    L1,
    L2,
  }
}

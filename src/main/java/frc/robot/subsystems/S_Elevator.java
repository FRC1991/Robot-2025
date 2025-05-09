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
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.handlers.CheckableSubsystem;
import frc.utils.Utils;
import frc.utils.Utils.ElasticUtil;

public class S_Elevator implements CheckableSubsystem {

  private boolean status = false;
  private boolean initialized = false;

  private SparkMax motor1, motor2;

  private PIDController posController;

  private static S_Elevator m_Instance;

  /** Creates a new Arm. */
  private S_Elevator() {
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

    motor1.getEncoder().setPosition(0);
    motor2.getEncoder().setPosition(0);

    posController = new PIDController(2, 0, 0);
    posController.setTolerance(ElevatorConstants.PID_ERROR_TOLERANCE);

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
  public static S_Elevator getInstance() {
    if(m_Instance == null) {
      m_Instance = new S_Elevator();
    }
    return m_Instance;
  }

  /**
   * @param speed The speed of the elevator [-1, 1]
   */
  public void set(double speed) {
    speed = Utils.normalize(speed);
    motor1.set(-speed);
    motor2.set(speed);
  }

  /**
   * Sets the internal PID controller to the given setpoint
   * @param setpoint A positional setpoint for the elevator to move towards
   */
  public void setSetpoint(double setpoint) {
    posController.setSetpoint(setpoint);
  }

  /**
   * Updates the speed of the motors based on their current position
   * moving towards the setpoint
   */
  public void moveToSetpoint() {
    set(posController.calculate(getEncoder()));
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
}

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
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.PivotConstants;
import frc.utils.Utils;
import frc.utils.Utils.ElasticUtil;

public class S_Pivot implements CheckableSubsystem {

  private boolean status = false;
  private boolean initialized = false;

  private SparkMax motor;

  private PIDController posController;

  private static S_Pivot m_Instance;

  private double p = 0.03, i = 0, d = 0;

  /** Creates a new Pivots. */
  private S_Pivot() {
    motor = new SparkMax(CANConstants.PIVOT_ID, MotorType.kBrushless);

    SparkMaxConfig pivotConfig = new SparkMaxConfig();

    pivotConfig.idleMode(IdleMode.kBrake)
      .smartCurrentLimit(Constants.NEO_CURRENT_LIMIT);

    motor.configure(pivotConfig, ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    ElasticUtil.putDouble("pivot P", () -> this.p, value -> {this.p=value;});
    ElasticUtil.putDouble("pivot I", () -> this.i, value -> {this.i=value;});
    ElasticUtil.putDouble("pivot D", () -> this.d, value -> {this.d=value;});
    ElasticUtil.putDouble("pivot encoder", () -> motor.getEncoder().getPosition());

    posController = new PIDController(p, i, d);
    posController.setTolerance(PivotConstants.PID_ERROR_TOLERANCE);

    motor.getEncoder().setPosition(0);

    initialized = true;
    status = true;
  }

  /**
   * @return The main Pivot object
   */
  public static S_Pivot getInstance() {
    if(m_Instance == null) {
      m_Instance = new S_Pivot();
    }
    return m_Instance;
  }

  public void zeroEncoder() {
    motor.getEncoder().setPosition(0);
  }

  /**
   * @param speed The speed of the pivot [-1, 1]
   */
  public void set(double speed) {
    motor.set(Utils.normalize(speed));
  }

  /**
   * Sets the internal PID controller to the given setpoint
   * @param setpoint A positional setpoint for the pivot to move towards
   */
  public void setSetpoint(double setpoint) {
    posController.setSetpoint(setpoint);
  }

  /**
   * Updates the speed of the motors based on their current position
   * moving towards the setpoint
   */
  public void moveToSetpoint() {
    set(posController.calculate(motor.getEncoder().getPosition()));
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
}

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

import frc.robot.Constants.CANConstants;
import frc.utils.Utils;
import frc.robot.Constants;

public class S_AlgaeIntake implements CheckableSubsystem {

  private boolean status = false;
  private boolean initialized = false;

  private SparkMax motor;

  private static S_AlgaeIntake m_Instance;

  /** Creates a new Algaes. */
  private S_AlgaeIntake() {
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
   * @return The main AlgaeIntake object
   */
  public static S_AlgaeIntake getInstance() {
    if(m_Instance == null) {
      m_Instance = new S_AlgaeIntake();
    }
    return m_Instance;
  }

  /**
   * @param speed The speed of the wheels on the intake [-1, 1]
   */
  public void set(double speed) {
    motor.set(Utils.normalize(speed));
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

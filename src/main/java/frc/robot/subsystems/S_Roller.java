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
import frc.robot.Constants;
import frc.robot.handlers.CheckableSubsystem;
import frc.utils.Utils;

public class S_Roller implements CheckableSubsystem {

  private boolean status = false;
  private boolean initialized = false;

  private SparkMax motor;
  
  private static S_Roller m_Instance;

  /** Creates a new S_Roller. */
  private S_Roller() {
    motor = new SparkMax(CANConstants.ROLLER_ID, MotorType.kBrushless);

    SparkMaxConfig S_RollersConfig = new SparkMaxConfig();

    S_RollersConfig.idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.NEO_CURRENT_LIMIT);

    motor.configure(S_RollersConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    initialized = true;
    status = true;
  }

  /**
   * @return The main Roller object
   */
  public static S_Roller getInstance() {
    if(m_Instance == null) {
      m_Instance = new S_Roller();
    }
    return m_Instance;
  }

  /**
   * @param speed The speed of the rollers [-1, 1]
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

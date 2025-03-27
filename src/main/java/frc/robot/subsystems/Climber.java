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
import frc.robot.Constants.ClimberConstants;
import frc.utils.Utils;
import frc.utils.Utils.ElasticUtil;

// This is for the coral
public class Climber extends SubsystemBase implements CheckableSubsystem, StateSubsystem {

  private boolean status = false;
  private boolean initialized = false;

  public SparkMax motor;

  private PIDController posController;

  private static Climber m_Instance;

  private double p = 0.5, i = 0, d = 0, pos = -300;

  private ClimberStates desiredState, currentState = ClimberStates.IDLE;

  /** Creates a new Climbers. */
  private Climber() {
    motor = new SparkMax(CANConstants.CLIMER_ID, MotorType.kBrushless);

    SparkMaxConfig climberConfig = new SparkMaxConfig();

    climberConfig.idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.NEO_CURRENT_LIMIT);

    motor.configure(climberConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    ElasticUtil.putDouble("climber P", () -> this.p, value -> {this.p=value;});
    ElasticUtil.putDouble("climber I", () -> this.i, value -> {this.i=value;});
    ElasticUtil.putDouble("climber D", () -> this.d, value -> {this.d=value;});
    ElasticUtil.putDouble("climber pos", () -> this.pos, value -> {this.pos=value;});
    ElasticUtil.putDouble("climber encoder", () -> motor.getEncoder().getPosition());

    posController = new PIDController(p, i, d);

    motor.getEncoder().setPosition(0);

    initialized = true;
    status = true;
  }

  /**
   * @return The main Climber object
   */
  public static Climber getInstance() {
    if(m_Instance == null) {
      m_Instance = new Climber();
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

  public void setDesiredState(ClimberStates state) {
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
      case IN:
        posController.setSetpoint(0);
        break;
      case OUT:
        posController.setSetpoint(pos);
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
      case IN:
      case OUT:
        motor.set(Utils.normalize(posController.calculate(motor.getEncoder().getPosition())));
        break;

      default:
        break;
    }

    if(!checkSubsystem()) {
      setDesiredState(ClimberStates.BROKEN);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * @return The current state of the subsystem
   */
  public ClimberStates getState() {
    return currentState;
  }

  public enum ClimberStates {
    IDLE,
    BROKEN,
    IN,
    OUT;
  }
}

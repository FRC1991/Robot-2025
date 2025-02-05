// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.AlgaeConstants;

// This is for the coral
public class AlgaeIntake extends SubsystemBase implements CheckableSubsystem, StateSubsystem {

  private boolean status = false;
  private boolean initialized = false;

  private SparkMax motor;

  private AlgaeStates desiredState, currentState = AlgaeStates.IDLE;

  /** Creates a new Algaes. */
  public AlgaeIntake() {
    motor = new SparkMax(CANConstants.ALGAE_INTAKE_ID, MotorType.kBrushless);

    initialized = true;
    status = true;
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

  public void setDesiredState(AlgaeStates state) {
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

  public enum AlgaeStates {
    IDLE,
    BROKEN,
    INTAKING,
    SCORING;
  }
}

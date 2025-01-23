// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

// This is for the coral
public class Pivot extends SubsystemBase implements CheckableSubsystem, StateSubsystem {

  private boolean status = false;
  private boolean initialized = false;

  private SparkMax motor;

  private PivotStates desiredState, currentState = PivotStates.IDLE;

  /** Creates a new Pivots. */
  public Pivot() {
    motor = new SparkMax(CANConstants.PIVOT_ID, MotorType.kBrushless);

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
        break;
      case SCORING:
        break;
      case INTAKING:
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
      case STORED:
        break;
      case SCORING:
        break;
      case INTAKING:
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

  public enum PivotStates {
    IDLE,
    BROKEN,
    STORED,
    SCORING,
    INTAKING;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.AlgaeIntake.AlgaeStates;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Pivot.PivotStates;
import frc.robot.subsystems.Roller.RollerStates;

public class Manager extends SubsystemBase implements CheckableSubsystem, StateSubsystem {

  private boolean status = false;
  private boolean initialized = false;

  private Pivot pivot = Pivot.getInstance();
  private Elevator elevator = Elevator.getInstance();
  private AlgaeIntake algaeIntake = AlgaeIntake.getInstance();
  private Roller roller = Roller.getInstance();

  private ManagerStates desiredState, currentState = ManagerStates.IDLE;

  /** Creates a new Manager. */
  public Manager() {
    // All subsystems should initialize when calling getInstance()
    initialized &= pivot.getInitialized();
    initialized &= elevator.getInitialized();
    initialized &= algaeIntake.getInitialized();
    initialized &= roller.getInitialized();
  }

  /**
   * Stops movement in all motors
   */
  @Override
  public void stop() {
    pivot.stop();
    elevator.stop();
    algaeIntake.stop();
    roller.stop();
  }

  /**
   * @return Has the constructor executed successfully
   */
  @Override
  public boolean getInitialized() {
    return initialized;
  }

  /**
   * @return Is the robot is okay to operate
   */
  @Override
  public boolean checkSubsystem() {
    status &= pivot.checkSubsystem();
    status &= elevator.checkSubsystem();
    status &= algaeIntake.checkSubsystem();
    status &= roller.checkSubsystem();

    return status;
  }

   /**
   * Updates any information the subsystem needs
   */
  @Override
  public void update() {
    pivot.update();
    elevator.update();
    algaeIntake.update();
    roller.update();

    switch(currentState) {
      case IDLE:
        // The robot should never be IDLE in a match
        setDesiredState(ManagerStates.DRIVE);
        break;
      case DRIVE:
        break;
      case ALGAE_INTAKE:
        break;
      case ALGAE_SCORE:
        break;
      case TAKEOFF:
        break;
      case SPIT:
        break;

      default:
        break;
    }
  }

  /**
   * Handles moving from one state to another. Also changes
   * the state of all subsystems to their respective states
   */
  @Override
  public void handleStateTransition() {
    switch(desiredState) {
      case IDLE:
        pivot.setDesiredState(PivotStates.IDLE);
        elevator.setDesiredState(ElevatorStates.IDLE);
        algaeIntake.setDesiredState(AlgaeStates.IDLE);
        roller.setDesiredState(RollerStates.IDLE);
        break;
      case DRIVE:
        pivot.setDesiredState(PivotStates.STORED);
        elevator.setDesiredState(ElevatorStates.STORED);
        algaeIntake.setDesiredState(AlgaeStates.IDLE);
        roller.setDesiredState(RollerStates.IDLE);
        break;
      case ALGAE_INTAKE:
        pivot.setDesiredState(PivotStates.INTAKING);
        elevator.setDesiredState(ElevatorStates.STORED);
        algaeIntake.setDesiredState(AlgaeStates.INTAKING);
        roller.setDesiredState(RollerStates.IDLE);
        break;
      case ALGAE_SCORE:
        pivot.setDesiredState(PivotStates.SCORING);
        elevator.setDesiredState(ElevatorStates.STORED);
        algaeIntake.setDesiredState(AlgaeStates.SCORING);
        roller.setDesiredState(RollerStates.IDLE);
        break;
      case TAKEOFF:
        pivot.setDesiredState(PivotStates.STORED);
        elevator.setDesiredState(ElevatorStates.L2);
        algaeIntake.setDesiredState(AlgaeStates.IDLE);
        roller.setDesiredState(RollerStates.INTAKING);
        break;
      case SPIT:
        pivot.setDesiredState(PivotStates.STORED);
        elevator.setDesiredState(ElevatorStates.STORED);
        algaeIntake.setDesiredState(AlgaeStates.IDLE);
        roller.setDesiredState(RollerStates.SCORING);
        break;
      

      default:
        break;
    }

    currentState = desiredState;
  }

  /**
   * Sets the desired state of the subsystem
   * @param state Desired state
   */
  public void setDesiredState(ManagerStates state) {
    if(this.desiredState != state) {
      desiredState = state;
      handleStateTransition();
    }
  }

  /**
   * @return The current state of the subsystem
   */
  public ManagerStates getState() {
    return currentState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * A Manager state integrates all of the other subsystem's states together
   * to create one cohesive state. This ensures that every subsystem is in
   * the correct state when performing an action. For example, when the
   * pivot is aiming, the shooter should be spinning up. When setting
   * the pivot state you may forget to spin up the shooter.
   *
   * <p>The "BROKEN" state which is present in all other subsystems
   * is not present here because it would mean that the whole
   * robot is broken. In that scenario the robot would be E-stopped
   * anyway and no code would run be running.
  */
  public enum ManagerStates {
    IDLE,
    /** Just driving the robot around */
    DRIVE,
    /** Running the AlgaeIntake to pull an Agae into our robot */
    ALGAE_INTAKE,
    /** Pushing the Algae out of our robot and into the processor */
    ALGAE_SCORE,
    TAKEOFF,
    SPIT,
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve.SwerveStates;

public class Manager extends SubsystemBase implements CheckableSubsystem, StateSubsystem {

  private boolean status = false;
  private boolean initialized = false;

  private Swerve swerve = Swerve.getInstance();
  private Spitter spitter = Spitter.getInstance();
  private Pivot pivot = Pivot.getInstance();
  private Elevator elevator = Elevator.getInstance();
  private AlgaeIntake algaeIntake = AlgaeIntake.getInstance();

  private ManagerStates desiredState, currentState = ManagerStates.IDLE;

  /** Creates a new Manager. */
  public Manager() {
    // All subsystems should initialize when calling getInstance()
    initialized = swerve.getInitialized();
    initialized &= spitter.getInitialized();
    initialized &= pivot.getInitialized();
    initialized &= elevator.getInitialized();
    initialized &= algaeIntake.getInitialized();
  }

  /**
   * Stops movement in all motors
   */
  @Override
  public void stop() {
    swerve.stop();
    spitter.stop();
    pivot.stop();
    elevator.stop();
    algaeIntake.stop();
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
    status = swerve.checkSubsystem();
    status &= spitter.checkSubsystem();
    status &= pivot.checkSubsystem();
    status &= elevator.checkSubsystem();
    status &= algaeIntake.checkSubsystem();

    return status;
  }

   /**
   * Updates any information the subsystem needs
   */
  @Override
  public void update() {
    swerve.update();
    spitter.update();
    pivot.update();
    elevator.update();
    algaeIntake.update();

    switch(currentState) {
      case IDLE:
        // The robot should never be IDLE in a match
        setDesiredState(ManagerStates.DRIVE);
        break;
      case DRIVE:
        break;
      case LOCKED:
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
        swerve.setDesiredState(SwerveStates.IDLE);
        break;
      case DRIVE:
        swerve.setDesiredState(SwerveStates.DRIVE);
        break;
      case LOCKED:
        swerve.setDesiredState(SwerveStates.LOCKED);
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
    /** Locking the wheels in an X formation */
    LOCKED,
    /** Running the AlgaeIntake to pull an Agae into our robot */
    ALGAE_INTAKE,
    /** Pushing the Algae out of our robot and into the processor */
    ALGAE_SCORE,
    /** Intaking coral from the coral station through the funnel */
    CORAL_INTAKE,
    /** Scoring coral in the L1 trough */
    CORAL_L1,
    /** Scoring coral on the L2 branch */
    CORAL_L2;
  }
}

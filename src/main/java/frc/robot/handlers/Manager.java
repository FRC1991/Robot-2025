// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.CheckableSubsystem;
import frc.robot.subsystems.S_AlgaeIntake;
import frc.robot.subsystems.S_Elevator;
import frc.robot.subsystems.S_Pivot;
import frc.robot.subsystems.S_Roller;
import frc.robot.handlers.AlgaeIntake.AlgaeStates;
import frc.robot.handlers.Elevator.ElevatorStates;
import frc.robot.handlers.Pivot.PivotStates;
import frc.robot.handlers.Roller.RollerStates;

public class Manager extends SubsystemBase implements CheckableSubsystem, StateSubsystem {

  private boolean status = false;
  private boolean initialized = false;

  private S_Pivot pivot = S_Pivot.getInstance();
  private S_Elevator elevator = S_Elevator.getInstance();
  private S_AlgaeIntake algaeIntake = S_AlgaeIntake.getInstance();
  private S_Roller roller = S_Roller.getInstance();

  private static Manager m_Instance;
  private ManagerStates desiredState, currentState = ManagerStates.IDLE;

  /** Creates a new Manager. */
  private Manager() {
    // All subsystems should initialize when calling getInstance()
    initialized &= pivot.getInitialized();
    initialized &= elevator.getInitialized();
    initialized &= algaeIntake.getInitialized();
    initialized &= roller.getInitialized();
  }

  /**
   * @return The main Swerve object
   */
  public static Manager getInstance() {
    if(m_Instance == null) {
      m_Instance = new Manager();
    }
    return m_Instance;
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
      case HOLD:
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
        Pivot.getInstance().setDesiredState(PivotStates.IDLE);
        Elevator.getInstance().setDesiredState(ElevatorStates.IDLE);
        AlgaeIntake.getInstance().setDesiredState(AlgaeStates.IDLE);
        Roller.getInstance().setDesiredState(RollerStates.IDLE);
        break;
      case DRIVE:
        Pivot.getInstance().setDesiredState(PivotStates.STORED);
        Elevator.getInstance().setDesiredState(ElevatorStates.STORED);
        AlgaeIntake.getInstance().setDesiredState(AlgaeStates.IDLE);
        Roller.getInstance().setDesiredState(RollerStates.IDLE);
        break;
      case ALGAE_INTAKE:
        Pivot.getInstance().setDesiredState(PivotStates.INTAKING);
        Elevator.getInstance().setDesiredState(ElevatorStates.STORED);
        AlgaeIntake.getInstance().setDesiredState(AlgaeStates.INTAKING);
        Roller.getInstance().setDesiredState(RollerStates.IDLE);
        break;
      case ALGAE_SCORE:
        Pivot.getInstance().setDesiredState(PivotStates.SCORING);
        Elevator.getInstance().setDesiredState(ElevatorStates.STORED);
        AlgaeIntake.getInstance().setDesiredState(AlgaeStates.SCORING);
        Roller.getInstance().setDesiredState(RollerStates.IDLE);
        break;
      case TAKEOFF:
        Pivot.getInstance().setDesiredState(PivotStates.STORED);
        Elevator.getInstance().setDesiredState(ElevatorStates.L2);
        AlgaeIntake.getInstance().setDesiredState(AlgaeStates.IDLE);
        Roller.getInstance().setDesiredState(RollerStates.INTAKING);
        break;
      case HOLD:
        Pivot.getInstance().setDesiredState(PivotStates.STORED);
        Elevator.getInstance().setDesiredState(ElevatorStates.STORED);
        AlgaeIntake.getInstance().setDesiredState(AlgaeStates.IDLE);
        Roller.getInstance().setDesiredState(RollerStates.INTAKING);
        break;
      case SPIT:
        Pivot.getInstance().setDesiredState(PivotStates.STORED);
        Elevator.getInstance().setDesiredState(ElevatorStates.STORED);
        AlgaeIntake.getInstance().setDesiredState(AlgaeStates.IDLE);
        Roller.getInstance().setDesiredState(RollerStates.SCORING);
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
  public void setDesiredState(State state) {
    if(this.desiredState != state) {
      desiredState = (ManagerStates) state;
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
    update();
  }

  /**
   * Binds a state to a button. This method helps improve
   * readability it the code by hiding all of the stuff with
   * the InstantCommands and just passing in the needed arguments.
   * 
   * @param button The Trigger (usually a button) to bind the states to
   * @param onTrue The state to be active while the button is held down
   * @param onFalse The state to be active one the button is released
   * @return Returns the new Trigger for further method chaining
   */
  public Trigger bindState(Trigger button, ManagerStates onTrue, ManagerStates onFalse) {
    return button
      .onTrue(new InstantCommand(() -> setDesiredState(onTrue), this))
      .onFalse(new InstantCommand(() -> setDesiredState(onFalse), this));
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
  public enum ManagerStates implements State {
    IDLE,
    /** Just driving the robot around */
    DRIVE,
    /** Running the AlgaeIntake to pull an Agae into our robot */
    ALGAE_INTAKE,
    /** Pushing the Algae out of our robot and into the processor */
    ALGAE_SCORE,
    TAKEOFF,
    SPIT,
    HOLD,
  }
}

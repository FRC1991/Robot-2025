package frc.robot.subsystems;

public interface StateSubsystem {

  /**
   * This should be called in the subsystem's periodic function.
   * Here is also a good place to implement subsystem checks if the subsystem
   * is implementing the CheckableSubsystem interface.
   */
  void update();

  /**
   * Handles transitions between states. For example, an above bumper intake
   * moving to an idle state from an active state will need to retract back into
   * the robot.
   */
  void handleStateTransition();

  /**
   * Changes the desired to the state passed into the method. Then it should
   * check that the current state does not match the desired state, otherwise it
   * should end. If the current state doesn't match the desired state, then call the
   * handleStateTransition method.
   * 
   * @param e This should be an Enum with all of the subsystem's states.
   */
  void setDesiredState(State e);

  /** IMPORTANT!
   * A subsystem's states should be defined in an enum implementing the interface below.
   * Every subsystem should have an idle state and broken state along with other 
   * subsystem specific states. A class implementing this interface should also have
   * a mutator and accessor method for the subsystem state.
   */
  interface State {}
}

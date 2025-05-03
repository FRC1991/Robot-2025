package frc.robot.subsystems;

public interface StateSubsystem {

  /* IMPORTANT!
   * A subsystem's states should be defined in an enum. Every subsystem should
   * have an idle state and broken state along with other subsystem specific states
   * A class implementing this interface should also have a mutator
   * and accessor method for the subsystem state.
   */

  /*
   * This should be used like a personalized periodic function for each state.
   * Here is also a good place to implement subsystem checks if the subsystem
   * is implementing the Checkable interface.
   */
  void update();

  /*
   * Handles transitions between states. For example, an above bumper intake
   * moving to an idle state from an active state will need to retract back into
   * the robot.
   */
  void handleStateTransition();

  void setDesiredState(State e);

  interface State {}
}

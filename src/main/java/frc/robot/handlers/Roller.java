package frc.robot.handlers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.S_Roller;

public class Roller extends SubsystemBase implements StateSubsystem {

  private RollerStates desiredState, currentState = RollerStates.IDLE;
  private S_Roller roller = S_Roller.getInstance();
  private static Roller m_Instance;

  private Roller() {}

  public static Roller getInstance() {
    if(m_Instance == null) {
      m_Instance = new Roller();
    }
    return m_Instance;
  }

  public void setDesiredState(State state) {
    if(this.desiredState != state) {
      desiredState = (RollerStates) state;
      handleStateTransition();
    }
  }

  @Override
  public void handleStateTransition() {
    switch(desiredState) {
      case IDLE:
        roller.stop();
        break;
      case BROKEN:
        roller.stop();
        break;
      case INTAKING:
        roller.set(RollerConstants.INTAKE_SPEED);
        break;
      case SCORING:
        roller.set(RollerConstants.SCORE_SPEED);
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

    if(!roller.checkSubsystem()) {
      setDesiredState(RollerStates.BROKEN);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    update();
  }

  /**
   * @return The current state of the subsystem
   */
  public RollerStates getState() {
    return currentState;
  }

  public enum RollerStates implements State {
    IDLE,
    BROKEN,
    INTAKING,
    SCORING;
  }
}

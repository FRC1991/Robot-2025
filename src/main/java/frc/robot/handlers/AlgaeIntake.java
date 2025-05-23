package frc.robot.handlers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.S_AlgaeIntake;

public class AlgaeIntake extends SubsystemBase implements StateSubsystem {

  private AlgaeStates desiredState, currentState = AlgaeStates.IDLE;
  private S_AlgaeIntake algaeIntake = S_AlgaeIntake.getInstance();
  private static AlgaeIntake m_Instance;
  
  private AlgaeIntake() {}

  public static AlgaeIntake getInstance() {
    if(m_Instance == null) {
      m_Instance = new AlgaeIntake();
    }
    return m_Instance;
  }

  public void setDesiredState(State state) {
    if(this.desiredState != state) {
      desiredState = (AlgaeStates) state;
      handleStateTransition();
    }
  }

  @Override
  public void handleStateTransition() {
    switch(desiredState) {
      case IDLE:
        algaeIntake.stop();
        break;
      case BROKEN:
        algaeIntake.stop();
        break;
      case INTAKING:
        algaeIntake.set(AlgaeConstants.INTAKE_SPEED);
        break;
      case SCORING:
        algaeIntake.set(AlgaeConstants.SCORING_SPEED);
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

    if(!algaeIntake.checkSubsystem()) {
      setDesiredState(AlgaeStates.BROKEN);
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
  public AlgaeStates getState() {
    return currentState;
  }

  public enum AlgaeStates implements State {
    IDLE,
    BROKEN,
    INTAKING,
    SCORING;
  }
}

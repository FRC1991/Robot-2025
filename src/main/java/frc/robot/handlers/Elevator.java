package frc.robot.handlers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.S_Elevator;

public class Elevator extends SubsystemBase implements StateSubsystem {

  private ElevatorStates desiredState, currentState = ElevatorStates.IDLE;
  private S_Elevator elevator = S_Elevator.getInstance();
  private static Elevator m_Instance;

  private Elevator() {}

  public static Elevator getInstance() {
    if(m_Instance == null) {
      m_Instance = new Elevator();
    }
    return m_Instance;
  }
  
  public void setDesiredState(State state) {
    if(this.desiredState != state) {
      desiredState = (ElevatorStates) state;
      handleStateTransition();
    }
  }

  @Override
  public void handleStateTransition() {
    switch(desiredState) {
      case IDLE:
        elevator.stop();
        break;
      case BROKEN:
        elevator.stop();
        break;
      case INTAKING:
        elevator.setSetpoint(ElevatorConstants.INTAKING_POSITION);
        break;
      case STORED:
        elevator.setSetpoint(ElevatorConstants.STORED_POSITION);
        break;
      case L1:
        elevator.setSetpoint(ElevatorConstants.L1_POSITION_INCHES);
        break;
      case L2:
        elevator.setSetpoint(ElevatorConstants.L2_POSITION_INCHES);
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
        elevator.moveToSetpoint();
        break;
      case STORED:
        elevator.moveToSetpoint();
        break;
      case L1:
        elevator.moveToSetpoint();
        break;
      case L2:
        elevator.moveToSetpoint();
        break;
        

      default:
        break;
    }

    if(!elevator.checkSubsystem()) {
      setDesiredState(ElevatorStates.BROKEN);
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
  public ElevatorStates getState() {
    return currentState;
  }

  public enum ElevatorStates implements State {
    IDLE,
    BROKEN,
    INTAKING,
    STORED,
    L1,
    L2,
  }
}

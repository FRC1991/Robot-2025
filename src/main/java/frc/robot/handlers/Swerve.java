package frc.robot.handlers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.OI;
import frc.robot.subsystems.S_Swerve;
import frc.utils.LimelightHelpers;
import frc.utils.Utils.ElasticUtil;

public class Swerve extends SubsystemBase implements StateSubsystem {

  private double driver = 0.6, lime = 0.4;

  private SwerveStates desiredState, currentState = SwerveStates.IDLE;
  private S_Swerve swerve = S_Swerve.getInstance();
  private static Swerve m_Instance;

  private Swerve() {
    ElasticUtil.putDouble("driver", () -> this.driver, value -> {this.driver=value;});
    ElasticUtil.putDouble("lime", () -> this.lime, value -> {this.lime=value;});
  }

  /**
   * @return The main Swerve object
   */
  public static Swerve getInstance() {
    if(m_Instance == null) {
      m_Instance = new Swerve();
    }
    return m_Instance;
  }

  @Override
  public void periodic() {
    update();
    
  }

  /**
   * Updates any information the subsystem needs
   */
  @Override
  public void update() {
    switch(currentState) {
      case IDLE:
        setDesiredState(SwerveStates.DRIVE);
        break;
      case BROKEN:
        break;
      case DRIVE:
        swerve.drive(
            OI.getDriveLeftY(),
            OI.getDriveLeftX(),
            OI.getDriveRightX(),
            true, SwerveConstants.SPEED_SCALE);
        break;
      case AIMING:
        switch((int) NetworkTableInstance.getDefault().getTable(Constants.LIMELIGHT_NAME).getEntry("tid").getInteger(-1)) {
          case 0:
            swerve.angleController.setSetpoint(0);
            break;
          case 1:
            swerve.angleController.setSetpoint(45);
            break;
          case 2:
            swerve.angleController.setSetpoint(90);
            break;
          case 3:
            swerve.angleController.setSetpoint(135);
            break;
          case 4:
            swerve.angleController.setSetpoint(180);
            break;
          case 5:
            swerve.angleController.setSetpoint(225);
            break;
          case 6:
            swerve.angleController.setSetpoint(270);
            break;
          case 7:
            swerve.angleController.setSetpoint(315);
            break;
        }
        swerve.drive(
          OI.getDriveLeftY(),
          OI.getDriveLeftX(),
          -swerve.angleController.calculate(swerve.getHeading()),
          true, SwerveConstants.SPEED_SCALE);
        break;
      case ALIGNING:
        swerve.drive(
          LimelightHelpers.getTA(Constants.LIMELIGHT_NAME) >= 1.7 ?
            -lime * MathUtil.applyDeadband(swerve.alignmentController.calculate(LimelightHelpers.getTX(Constants.LIMELIGHT_NAME), 0), OIConstants.DRIVER_DEADBAND)
            + driver * OI.getDriveLeftY()
          :
            OI.getDriveLeftY()
          ,
          OI.getDriveLeftX(),
          -swerve.angleController.calculate(swerve.getHeading(), 270),
          true, 0.8);
        break;
      case LOCKED:
        swerve.setX();
        break;

      default:
        break;
    }

    // if(!checkSubsystem()) {
    //   setDesiredState(SwerveStates.BROKEN);
    // }
  }

  /**
   * Handles moving from one state to another
   */
  @Override
  public void handleStateTransition() {
    switch(desiredState) {
      case IDLE:
        swerve.stop();
        break;
      case BROKEN:
        swerve.stop();
        break;
      case DRIVE:
        break;
      case AIMING:
        break;
      case ALIGNING:
        break;
      case LOCKED:
        swerve.setX();
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
      desiredState = (SwerveStates) state;
      handleStateTransition();
    }
  }

  /**
   * @return The current state of the subsystem
   */
  public SwerveStates getState() {
    return currentState;
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
  public Trigger bindState(Trigger button, SwerveStates onTrue, SwerveStates onFalse) {
    return button
      .onTrue(new InstantCommand(() -> setDesiredState(onTrue), this))
      .onFalse(new InstantCommand(() -> setDesiredState(onFalse), this));
  }

  /**
   * The list of possible states for this subsystem
   */
  public enum SwerveStates implements State {
    IDLE,
    BROKEN,
    /** Regular control of the robot */
    DRIVE,
    /** Aims towards the angle of the April tags around the field.
     * This takes away yaw control from the driver.
     */
    AIMING,
    ALIGNING,
    /** Removes all control and locks the wheels in an X formation */
    LOCKED;
  }
}

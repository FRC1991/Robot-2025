// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.handlers.AlgaeIntake;
import frc.robot.handlers.Manager;
import frc.robot.handlers.Manager.ManagerStates;
import frc.robot.handlers.Pivot;
import frc.robot.handlers.Roller;
import frc.robot.handlers.Swerve;
import frc.robot.subsystems.S_Swerve;
import frc.robot.handlers.AlgaeIntake.AlgaeStates;
import frc.robot.handlers.Elevator;
import frc.robot.handlers.Swerve.SwerveStates;
import frc.robot.subsystems.S_Pivot;
import frc.utils.Utils.ElasticUtil;

public class RobotContainer {

  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();
    configureElastic();
  }

  /**
   * Configures the auto chooser and other info. Most information is published
   * and read from inside different subsystems.
   */
  private void configureElastic() {
    // This is from the Elastic documentation. I think it is for saving/reading layouts
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    
    // Current state of each subsystem and Manager
    ElasticUtil.putString("Manager State", () -> Manager.getInstance().getState().toString());
    ElasticUtil.putString("Swerve State", () -> Swerve.getInstance().getState().toString());
    ElasticUtil.putString("Pivot State", () -> Pivot.getInstance().getState().toString());
    ElasticUtil.putString("Elevator State", () -> Elevator.getInstance().getState().toString());
    ElasticUtil.putString("AlgaeIntake State", () -> AlgaeIntake.getInstance().getState().toString());
    ElasticUtil.putString("Roller State", () -> Roller.getInstance().getState().toString());

    autoChooser = new SendableChooser<Command>();

    RunCommand leave = new RunCommand(() -> S_Swerve.getInstance().drive(-0.5, 0, 0, false), Swerve.getInstance());
    
    autoChooser.addOption("leave thing", leave);

    SequentialCommandGroup coralOne = new SequentialCommandGroup(
      // Driving up to the reef with the ground intake facing the reef
      new RunCommand(
        () -> S_Swerve.getInstance().drive(-0.25, 0, 0, false), Swerve.getInstance()
      ).withTimeout(2),
      // Spitting the coral off of the robot and into the trough
      new InstantCommand(
        () -> AlgaeIntake.getInstance().setDesiredState(AlgaeStates.INTAKING), AlgaeIntake.getInstance()
      ).withTimeout(1),
      // Waiting for the coral to settle
      new WaitCommand(Time.ofRelativeUnits(2, Units.Seconds)),
      // Driving away from the reef, so the robot doesn't support the coral at the end of auto
      new RunCommand(
        () -> S_Swerve.getInstance().drive(0.3, 0, 0, false), Swerve.getInstance()
      ).withTimeout(0.25),
      // Setting the state to DRIVE for the start of teleop
      new InstantCommand(
        () -> Manager.getInstance().setDesiredState(ManagerStates.DRIVE), Manager.getInstance()
      )
    );

    autoChooser.addOption("coral uno por favor", coralOne);

    SmartDashboard.putData("Auto", autoChooser);
  }

  /**
   * Sets all of the controls for both the driver and the aux driver
   */
  private void configureBindings() {
    OI.auxController.povUp()
      .whileTrue(new RunCommand(() -> S_Pivot.getInstance().set(0.3), Pivot.getInstance()))
      .onFalse(new InstantCommand(() -> S_Pivot.getInstance().zeroEncoder()));

    OI.auxController.povDown()
      .whileTrue(new RunCommand(() -> S_Pivot.getInstance().set(-0.3), Pivot.getInstance()))
      .onFalse(new InstantCommand(() -> S_Pivot.getInstance().zeroEncoder()));

    Manager.getInstance().bindState(OI.auxController.a(), ManagerStates.TAKEOFF, ManagerStates.HOLD);

    Manager.getInstance().bindState(OI.auxController.b(), ManagerStates.SPIT, ManagerStates.DRIVE);

    Manager.getInstance().bindState(OI.auxController.rightBumper(), ManagerStates.ALGAE_SCORE, ManagerStates.DRIVE);

    Manager.getInstance().bindState(OI.auxController.leftBumper(), ManagerStates.ALGAE_INTAKE, ManagerStates.DRIVE);

    // Stops movement by setting the wheels in an X formation
    Swerve.getInstance().bindState(OI.driverController.rightBumper(), SwerveStates.LOCKED, SwerveStates.DRIVE);
    
    // Removes yaw control and aligns to April tags
    Swerve.getInstance().bindState(OI.driverController.a(), SwerveStates.AIMING, SwerveStates.DRIVE);

    // Removes yaw control, faces the processor, and nudges the robot towards the center
    Swerve.getInstance().bindState(OI.driverController.leftBumper(), SwerveStates.ALIGNING, SwerveStates.DRIVE);
    
    // Zeroes out the gyro
    OI.driverController.start()
      .onTrue(new InstantCommand(() -> S_Swerve.getInstance().setHeading(0), Swerve.getInstance()));
  }

  /**
   * @return The currently selected command from the auto chooser
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

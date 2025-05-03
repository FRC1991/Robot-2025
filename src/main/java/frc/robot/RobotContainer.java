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
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manager;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.AlgaeIntake.AlgaeStates;
import frc.robot.subsystems.Manager.ManagerStates;
import frc.robot.subsystems.Swerve.SwerveStates;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Roller;
import frc.utils.Utils.ElasticUtil;

public class RobotContainer {

  private SendableChooser<Command> autoChooser;

  public final Manager m_Manager = new Manager();

  public RobotContainer() {
    configureBindings();
    configureElastic();
  }

  private void configureElastic() {
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    
    ElasticUtil.putString("Manager State", () -> m_Manager.getState().toString());
    ElasticUtil.putString("Swerve State", () -> Swerve.getInstance().getState().toString());
    ElasticUtil.putString("Pivot State", () -> Pivot.getInstance().getState().toString());
    ElasticUtil.putString("Elevator State", () -> Elevator.getInstance().getState().toString());
    ElasticUtil.putString("AlgaeIntake State", () -> AlgaeIntake.getInstance().getState().toString());
    ElasticUtil.putString("Roller State", () -> Roller.getInstance().getState().toString());

    autoChooser = new SendableChooser<Command>();

    RunCommand leave = new RunCommand(() -> Swerve.getInstance().drive(-0.5, 0, 0, false), Swerve.getInstance());
    
    autoChooser.addOption("leave thing", leave);

    SequentialCommandGroup coralOne = new SequentialCommandGroup(
      new RunCommand(
        () -> Swerve.getInstance().drive(-0.25, 0, 0, false), Swerve.getInstance()
      ).withTimeout(2),
      new InstantCommand(
        () -> AlgaeIntake.getInstance().setDesiredState(AlgaeStates.INTAKING), AlgaeIntake.getInstance()
      ).withTimeout(1),
      new WaitCommand(Time.ofRelativeUnits(2, Units.Seconds)),
      new RunCommand(
        () -> Swerve.getInstance().drive(0.3, 0, 0, false), Swerve.getInstance()
      ).withTimeout(0.5),
      new InstantCommand(
        () -> m_Manager.setDesiredState(ManagerStates.DRIVE), m_Manager
      )
    );

    autoChooser.addOption("coral uno por favor", coralOne);

    SmartDashboard.putData("Auto", autoChooser);
  }

  private void configureBindings() {
    OI.auxController.povUp()
      .whileTrue(new RunCommand(() -> Pivot.getInstance().motor.set(0.3), Pivot.getInstance()))
      .onFalse(new InstantCommand(() -> Pivot.getInstance().motor.getEncoder().setPosition(0)));

    OI.auxController.povDown()
      .whileTrue(new RunCommand(() -> Pivot.getInstance().motor.set(-0.3), Pivot.getInstance()))
      .onFalse(new InstantCommand(() -> Pivot.getInstance().motor.getEncoder().setPosition(0)));

    m_Manager.bindState(OI.auxController.a(), ManagerStates.TAKEOFF, ManagerStates.HOLD);

    m_Manager.bindState(OI.auxController.b(), ManagerStates.SPIT, ManagerStates.DRIVE);

    m_Manager.bindState(OI.auxController.rightBumper(), ManagerStates.ALGAE_SCORE, ManagerStates.DRIVE);

    m_Manager.bindState(OI.auxController.leftBumper(), ManagerStates.ALGAE_INTAKE, ManagerStates.DRIVE);

    // Stops movement by setting the wheels in an X formation
    Swerve.getInstance().bindState(OI.driverController.rightBumper(), SwerveStates.LOCKED, SwerveStates.DRIVE);
    
    // Removes yaw control and aligns to April tags
    Swerve.getInstance().bindState(OI.driverController.a(), SwerveStates.AIMING, SwerveStates.DRIVE);

    // Removes yaw control, faces the processor, and nudges the robot towards the center
    Swerve.getInstance().bindState(OI.driverController.leftBumper(), SwerveStates.ALIGNING, SwerveStates.DRIVE);
    
    // Zeroes out the gyro
    OI.driverController.start()
      .onTrue(new InstantCommand(() -> Swerve.getInstance().setHeading(0), Swerve.getInstance()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

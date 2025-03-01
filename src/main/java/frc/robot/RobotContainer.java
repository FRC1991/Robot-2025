// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manager;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Manager.ManagerStates;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Spitter;
import frc.utils.Utils.ElasticUtil;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  private final Manager m_Manager = new Manager();

  public RobotContainer() {
    // This would throw an error no matter what
    // in last year's code, so let's hope for better
    // this year.
    autoChooser = AutoBuilder.buildAutoChooser();

    configureElastic();
    configureBindings();
  }

  private void configureElastic() {
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    
    ElasticUtil.putString("Manager State", () -> m_Manager.getState().toString());
    ElasticUtil.putString("Swerve State", () -> Swerve.getInstance().getState().toString());
    ElasticUtil.putString("Spitter State", () -> Spitter.getInstance().getState().toString());
    ElasticUtil.putString("Pivote State", () -> Pivot.getInstance().getState().toString());
    ElasticUtil.putString("Elevator State", () -> Elevator.getInstance().getState().toString());
    ElasticUtil.putString("AlgaeIntake State", () -> AlgaeIntake.getInstance().getState().toString());
  }

  private void configureBindings() {
    m_Manager.setDefaultCommand(new RunCommand(() -> m_Manager.update(), m_Manager));

    OI.driverController.y()
        .onTrue(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.CORAL_L1), m_Manager))
        .onFalse(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE), m_Manager));
    OI.driverController.b()
        .onTrue(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.CORAL_L2), m_Manager))
        .onFalse(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE), m_Manager));

    OI.driverController.x()
        .onTrue(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.CORAL_INTAKE), m_Manager))
        .onFalse(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE), m_Manager));
    
    OI.driverController.rightBumper()
        .onTrue(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.ALGAE_SCORE), m_Manager))
        .onFalse(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE), m_Manager));

    OI.driverController.leftBumper()
        .onTrue(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.ALGAE_INTAKE), m_Manager))
        .onFalse(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE), m_Manager));
    
    // Stops movement by setting the wheels in an X formation
    OI.driverController.a()
        .onTrue(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.LOCKED), m_Manager))
        .onFalse(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE), m_Manager));
    
    // Zeroes out the gyro
    OI.driverController.start()
      .onTrue(new InstantCommand(() -> Swerve.getInstance().zeroHeading(), m_Manager));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

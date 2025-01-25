// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Manager;
import frc.robot.subsystems.Manager.ManagerStates;

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
    // The main tab is used during a match to display relavent information
    ShuffleboardTab main = Shuffleboard.getTab("Main");
    main.addString("Manager State", () -> m_Manager.getState().toString());
    main.addString("Swerve State", () -> m_Manager.swerve.getState().toString());
  }

  private void configureBindings() {
    m_Manager.setDefaultCommand(new RunCommand(() -> m_Manager.update(), m_Manager));

    // Stops movement - Works
    new JoystickButton(OI.driverJoytick, 1)
        .onTrue(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.LOCKED), m_Manager))
        .onFalse(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE), m_Manager));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

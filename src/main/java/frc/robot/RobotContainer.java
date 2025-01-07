// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Swerve;

public class RobotContainer {

  private Swerve m_swerve = Swerve.getInstance();

  public RobotContainer() {

    configureBindings();
  }

  private void configureBindings() {
    m_swerve.setDefaultCommand(new RunCommand(() -> m_swerve.update(), m_swerve));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

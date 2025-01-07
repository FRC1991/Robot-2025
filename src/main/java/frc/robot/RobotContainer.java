// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Manager;
import frc.robot.subsystems.Swerve;

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
    // Not sure what this does, but the docs say I need it
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    SmartDashboard.putData("Swerve Drive", new Sendable() {
        @Override
        public void initSendable(SendableBuilder builder) {
          builder.setSmartDashboardType("SwerveDrive");

          builder.addDoubleProperty("Front Left Angle", () -> Swerve.getInstance().getModuleStates()[0].angle.getRadians(), null);
          builder.addDoubleProperty("Front Left Velocity", () -> Swerve.getInstance().getModuleStates()[0].speedMetersPerSecond, null);

          builder.addDoubleProperty("Front Right Angle", () -> Swerve.getInstance().getModuleStates()[1].angle.getRadians(), null);
          builder.addDoubleProperty("Front Right Velocity", () -> Swerve.getInstance().getModuleStates()[1].speedMetersPerSecond, null);

          builder.addDoubleProperty("Back Left Angle", () -> Swerve.getInstance().getModuleStates()[2].angle.getRadians(), null);
          builder.addDoubleProperty("Back Left Velocity", () -> Swerve.getInstance().getModuleStates()[2].speedMetersPerSecond, null);

          builder.addDoubleProperty("Back Right Angle", () -> Swerve.getInstance().getModuleStates()[3].angle.getRadians(), null);
          builder.addDoubleProperty("Back Right Velocity", () -> Swerve.getInstance().getModuleStates()[3].speedMetersPerSecond, null);

          builder.addDoubleProperty("Robot Angle", () -> Swerve.getInstance().getHeading(), null);
        }
    });
  }

  private void configureBindings() {
    m_Manager.setDefaultCommand(new RunCommand(() -> m_Manager.update(), m_Manager));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

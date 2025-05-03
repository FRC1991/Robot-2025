// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Manager.ManagerStates;
import frc.robot.subsystems.Roller.RollerStates;
import frc.robot.subsystems.Swerve.SwerveStates;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    Roller.getInstance().setDesiredState(RollerStates.INTAKING);
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    try {
      Thread.sleep(100);
    } catch (InterruptedException e) {
      Roller.getInstance().setDesiredState(RollerStates.IDLE);
    }
    Roller.getInstance().setDesiredState(RollerStates.IDLE);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.m_Manager.setDesiredState(ManagerStates.DRIVE);
    Swerve.getInstance().setDesiredState(SwerveStates.DRIVE);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}

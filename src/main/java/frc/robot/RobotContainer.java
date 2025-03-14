// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.features2d.AgastFeatureDetector;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manager;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Manager.ManagerStates;
import frc.robot.subsystems.Pivot.PivotStates;
import frc.robot.subsystems.Swerve.SwerveStates;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Spitter;
import frc.utils.Utils.ElasticUtil;

public class RobotContainer {

  private SendableChooser<Command> autoChooser;
//   private SendableChooser<Command> autoTwo;

  private final Manager m_Manager = new Manager();

  public RobotContainer() {
    NamedCommands.registerCommand("CORAL_L2", new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.CORAL_L2), m_Manager));
    NamedCommands.registerCommand("DRIVE", new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE), m_Manager));
    NamedCommands.registerCommand("ALGAE_SCORE", new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.ALGAE_SCORE), m_Manager));
    NamedCommands.registerCommand("IDLE", new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.IDLE), m_Manager));

    configureBindings();
    configureElastic();
  }

  private void configureElastic() {
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    
    ElasticUtil.putString("Manager State", () -> m_Manager.getState().toString());
    ElasticUtil.putString("Swerve State", () -> Swerve.getInstance().getState().toString());
    ElasticUtil.putString("Spitter State", () -> Spitter.getInstance().getState().toString());
    ElasticUtil.putString("Pivote State", () -> Pivot.getInstance().getState().toString());
    ElasticUtil.putString("Elevator State", () -> Elevator.getInstance().getState().toString());
    ElasticUtil.putString("AlgaeIntake State", () -> AlgaeIntake.getInstance().getState().toString());

    // This would throw an error no matter what
    // in last year's code, so let's hope for better
    // this year.
    autoChooser = AutoBuilder.buildAutoChooser();

    RunCommand leave = new RunCommand(() -> Swerve.getInstance().drive(0.5, 0, 0, false), Swerve.getInstance());
    autoChooser.addOption("leave thing", leave);

    SequentialCommandGroup coralOne = new SequentialCommandGroup(
        new RunCommand(
            () -> Swerve.getInstance().drive(0.8, 0, 0, false), Swerve.getInstance()
        ).withTimeout(3),
        new InstantCommand(
            () -> m_Manager.setDesiredState(ManagerStates.CORAL_L2), m_Manager
        ).withTimeout(5)
    );
    autoChooser.addOption("coral uno por favor", coralOne);

    SmartDashboard.putData("pathplanner chooser", autoChooser);
    
    // autoTwo = new SendableChooser<Command>();
    // autoTwo.addOption("leave B auto", new PathPlannerAuto("Leave(B)-a"));
    // autoTwo.addOption("leave B path", PathPlannerPath.fromPathFile("Leave(B)"));
    // SmartDashboard.putData("manual chooser", autoTwo);
  }

  private void configureBindings() {
    m_Manager.setDefaultCommand(new RunCommand(() -> m_Manager.update(), m_Manager));
    Swerve.getInstance().setDefaultCommand(new RunCommand(() -> Swerve.getInstance().update(), Swerve.getInstance()));

    Pivot.getInstance().setDefaultCommand(new RunCommand(() -> Pivot.getInstance().motor.set(MathUtil.applyDeadband(-OI.auxController.getRightY(), 0.07)), Pivot.getInstance()));

    OI.auxController.y()
        .onTrue(new InstantCommand(() -> Pivot.getInstance().motor.getEncoder().setPosition(0)));

    // Swerve.getInstance().setDesiredState(SwerveStates.DRIVE);
    // m_Manager.setDesiredState(ManagerStates.DRIVE);

    // OI.auxController.y()
    //     .onTrue(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.CORAL_L1), m_Manager))
    //     .onFalse(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE), m_Manager));
    
    OI.auxController.b()
        .onTrue(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.CORAL_L2), m_Manager))
        .onFalse(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE), m_Manager));

    OI.auxController.x()
        .onTrue(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.CORAL_INTAKE), m_Manager))
        .onFalse(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE), m_Manager));
    
    OI.auxController.rightBumper()
        .onTrue(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.ALGAE_SCORE), m_Manager))
        .onFalse(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE), m_Manager));

    OI.auxController.leftBumper()
        .onTrue(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.ALGAE_INTAKE), m_Manager))
        .onFalse(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE), m_Manager));
    
    // Stops movement by setting the wheels in an X formation
    OI.driverController.rightBumper()
        .onTrue(new InstantCommand(() -> Swerve.getInstance().setDesiredState(SwerveStates.LOCKED), Swerve.getInstance()))
        .onFalse(new InstantCommand(() -> Swerve.getInstance().setDesiredState(SwerveStates.DRIVE), Swerve.getInstance()));
    
    // Removes yaw control and aligns to April tags
    // OI.driverController.a()
    //     .onTrue(new InstantCommand(() -> Swerve.getInstance().setDesiredState(SwerveStates.AIMING), Swerve.getInstance()))
    //     .onFalse(new InstantCommand(() -> Swerve.getInstance().setDesiredState(SwerveStates.DRIVE), Swerve.getInstance()));

    OI.driverController.a()
        .onTrue(new InstantCommand(() -> Swerve.getInstance().setDesiredState(SwerveStates.ALIGNING), Swerve.getInstance()))
        .onFalse(new InstantCommand(() -> Swerve.getInstance().setDesiredState(SwerveStates.DRIVE), Swerve.getInstance()));

    OI.driverController.leftBumper()
        .onTrue(new InstantCommand(() -> Swerve.getInstance().setDesiredState(SwerveStates.MANUAL), Swerve.getInstance()))
        .onFalse(new InstantCommand(() -> Swerve.getInstance().setDesiredState(SwerveStates.DRIVE), Swerve.getInstance()));
    
    // Zeroes out the gyro
    OI.driverController.start()
        .onTrue(new InstantCommand(() -> Swerve.getInstance().setHeading(0), Swerve.getInstance()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

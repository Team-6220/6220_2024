// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

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
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
            Constants.isRed = "red";
        }
        if (ally.get() == Alliance.Blue) {
            Constants.isRed = "blue";
        }
    }
    else {
        Constants.isRed = "N/A";
    }
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
            Constants.isRed = "red";
        }
        if (ally.get() == Alliance.Blue) {
            Constants.isRed = "blue";
        }
    }
    else {
        Constants.isRed = "N/A";
    }
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
            Constants.isRed = "red";
        }
        if (ally.get() == Alliance.Blue) {
            Constants.isRed = "blue";
        }
    }
    else {
        Constants.isRed = "N/A";
    }
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}

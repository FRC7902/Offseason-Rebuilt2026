// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.shooter.launch_calculator.LaunchCalculator;
import frc.robot.utils.AutoHelper;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Clear launching parameters
    LaunchCalculator.getInstance().clearLaunchingParameters();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    String selectedAutoName = m_robotContainer.getAutonomousCommand().getName();

    if (!selectedAutoName.equals("InstantCommand")) {
      AutoHelper.publishAutoPath(m_robotContainer.getAutonomousCommand().getName());
    }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    teleopAndAutonomousInit();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    teleopAndAutonomousInit();

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
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  private void teleopAndAutonomousInit() {
    // TODO: Re-enable this once limit switches are tested
    // Check the linear intake position and set the encoder position accordingly
    // m_robotContainer.calibrateLinearIntakePosition();

    // CommandScheduler.getInstance().schedule(m_robotContainer.stopAllSubsystems());

    // Start the flywheel at the default RPM when teleop starts
    // CommandScheduler.getInstance().schedule(m_robotContainer.m_shooterSubsystem.startFlywheelDefaultRPM());
  }

  @Override
  public void simulationPeriodic() {
    m_robotContainer.publishComponentPoses();
  }
}

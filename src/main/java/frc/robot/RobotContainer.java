// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.IndexerSystem;
import frc.robot.subsystems.intake.IntakeSystem;
import frc.robot.subsystems.shooter.ShooterSystem;

public class RobotContainer {

    IndexerSystem m_indexerSystem;
    IntakeSystem m_intakeSystem;
    ShooterSystem m_shooterSystem;

    public RobotContainer() {

        // Start data logging
        DataLogManager.start();
        // Include DriverStation data in the log
        DriverStation.startDataLog(DataLogManager.getLog());

        m_indexerSystem = new IndexerSystem();
        m_intakeSystem = new IntakeSystem();
        m_shooterSystem = new ShooterSystem();

        configureBindings();
    }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}

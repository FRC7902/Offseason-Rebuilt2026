package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;

public class ShooterSystem {

  private final FlywheelSubsystem m_flywheel;
  private final HoodSubsystem m_hood;
  private final TurretSubsystem m_turret;

  public ShooterSystem(FlywheelSubsystem flywheel, HoodSubsystem hood, TurretSubsystem turret) {
    m_flywheel = flywheel;
    m_hood = hood;
    m_turret = turret;

    // TODO: Set up default command for turret to continuously track target
  }

  // TODO: Make fake launchCalculator

  public Command aimAndShoot() {
    throw new UnsupportedOperationException("Not yet implemented.");
  }
}

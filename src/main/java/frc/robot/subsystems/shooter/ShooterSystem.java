package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.launch_calculator.LaunchCalculator;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;

public class ShooterSystem {

  private final FlywheelSubsystem m_flywheel;
  private final HoodSubsystem m_hood;
  private final TurretSubsystem m_turret;

  public ShooterSystem(FlywheelSubsystem flywheel, HoodSubsystem hood, TurretSubsystem turret) {
    m_flywheel = flywheel;
    m_hood = hood;
    m_turret = turret;

    // TODO: Set the default command for the turret to aim at the target
  }

  public Command aimAndShoot() {
    final var parameters = LaunchCalculator.getInstance().getParameters();

    return Commands.sequence(
        m_hood.setAngle(parameters.hoodAngle()),
        m_flywheel.setVelocity(parameters.flywheelSpeed()));
  }

  public Command aimTurret() {
    // LaunchCalculator.getInstance().getParameters().turretAngle();
    throw new UnsupportedOperationException("Not yet implemented.");
  }

  // Returns true if the shooter is ready to fire (flywheel at speed, hood at
  // angle, turret aimed)
  public boolean isShooterReady() {
    throw new UnsupportedOperationException("Not yet implemented.");
  }
}

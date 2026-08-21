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

    // TODO: Set the default command for the turret to aim at the target
  }

  /**
   * Creates a command that sets the flywheel speed and hood angle based on the current launch
   * calculator parameters.
   *
   * @return command that indefinitely aims the shooter and runs the flywheel
   */
  public Command aimAndShoot() {
    // final var parameters = LaunchCalculator.getInstance().getParameters();

    // parameters.hoodAngle();
    // parameters.flywheelSpeed();

    throw new UnsupportedOperationException("Not yet implemented.");
  }

  /**
   * Creates a command that sets the turret angle based on the current launch calculator parameters.
   *
   * @return command that indefinitely aims the turret to the calculated target angle
   */
  public Command aimTurret() {
    // LaunchCalculator.getInstance().getParameters().turretAngle();
    throw new UnsupportedOperationException("Not yet implemented.");
  }

  /**
   * Returns whether the shooter is ready to fire based on flywheel speed, hood angle, and turret
   * alignment.
   *
   * @return true if the flywheel is at speed, the hood is at the target angle, and the turret is
   *     aimed correctly
   */
  public boolean isShooterReady() {
    throw new UnsupportedOperationException("Not yet implemented.");
  }
}

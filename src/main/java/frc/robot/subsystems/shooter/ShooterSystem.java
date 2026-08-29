package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.launch_calculator.LaunchCalculator;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;

public class ShooterSystem extends SubsystemBase {

  private final FlywheelSubsystem m_flywheel;
  private final HoodSubsystem m_hood;
  private final TurretSubsystem m_turret;

  public ShooterSystem(FlywheelSubsystem flywheel, HoodSubsystem hood, TurretSubsystem turret) {
    m_flywheel = flywheel;
    m_hood = hood;
    m_turret = turret;

    m_turret.setDefaultCommand(aimTurret());
  }

  /**
   * Creates a command that sets the flywheel speed and hood angle based on the current launch
   * calculator parameters.
   *
   * @return command that indefinitely aims the shooter and runs the flywheel
   */
  public Command aimAndShoot() {
    final var launchCalculator = LaunchCalculator.getInstance();

    return Commands.parallel(
            m_hood.setAngle(() -> launchCalculator.getParameters().hoodAngle()),
            m_flywheel.setVelocity(() -> launchCalculator.getParameters().flywheelSpeed()))
        .repeatedly();
  }

  /**
   * Creates a command that sets the turret angle based on the current launch calculator parameters.
   *
   * @return command that indefinitely aims the turret to the calculated target angle
   */
  public Command aimTurret() {
    return m_turret
        .setAngle(() -> LaunchCalculator.getInstance().getParameters().turretAngle())
        .repeatedly();
  }

  /**
   * Returns whether the shooter is ready to fire based on flywheel speed, hood angle, and turret
   * alignment.
   *
   * @return true if the flywheel is at speed, the hood is at the target angle, and the turret is
   *     aimed correctly
   */
  public boolean isShooterReady() {
    return m_flywheel.isAtSetpoint() && m_hood.isAtSetpoint() && m_turret.isAtSetpoint();
  }

  /**
   * Creates a command that stops the flywheel, hood, and turret subsystems.
   *
   * @return command that stops all shooter subsystems and runs indefinitely until interrupted
   */
  public Command stopShooting() {
    // TODO: Change behaviour to slow flyweheel down to default speed, and lower hood to safe angle
    return Commands.parallel(m_flywheel.stop(), m_hood.stop());
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("ShooterSystem/isShooterReady", isShooterReady());
  }
}

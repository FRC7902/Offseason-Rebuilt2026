package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.launch_calculator.LaunchCalculator;
import frc.robot.subsystems.shooter.turret.TurretConstants;
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

  public Command aimAndShoot() {
    final var launchCalculator = LaunchCalculator.getInstance();

    return Commands.parallel(
        m_hood.setAngle(() -> launchCalculator.getParameters().hoodAngle()),
        m_flywheel.setVelocity(() -> launchCalculator.getParameters().flywheelSpeed()));
  }

  public Command manualAimAndShoot() {
    return Commands.parallel(

        m_turret.setAngle(TurretConstants.DEFAULT_ANGLE).repeatedly(),
        m_hood.setAngle(HoodConstants.DEFAULT_ANGLE),
        m_flywheel.setVelocity(FlywheelConstants.DEFAULT_RPM));
  }

  public Command flywheelDefaultSpeed() {
    return m_flywheel.setVelocity(FlywheelConstants.DEFAULT_RPM);
  }

  public Command aimTurret() {
    return Commands.run(
        () ->
            m_turret.setAngleSetpoint(LaunchCalculator.getInstance().getParameters().turretAngle()),
        m_turret);
  }

  public boolean isShooterReady() {
    return m_flywheel.isAtSetpoint() && m_hood.isHoodReadyToShoot() && m_turret.isAtSetpoint();
  }

  public boolean isTurretReady() {
    return m_turret.isAtSetpoint();
  }

  public Command reverseFlywheel() {
    return m_flywheel.setDutyCycle(-0.25);
  }

  public Command stop() {

    return Commands.parallel(
        m_flywheel.setVelocity(FlywheelConstants.DEFAULT_RPM),
        m_hood.setAngle(HoodConstants.MIN_ANGLE));
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("ShooterSystem/isShooterReady", isShooterReady());
  }
}

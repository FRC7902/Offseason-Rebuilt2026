package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;

public class ShooterSystem extends SubsystemBase {

    private final FlywheelSubsystem m_flywheel;
    private final HoodSubsystem m_hood;
    private final TurretSubsystem m_turret;

    public ShooterSystem() {
        m_flywheel = new FlywheelSubsystem();
        m_hood = new HoodSubsystem();
        m_turret = new TurretSubsystem();
    }
}

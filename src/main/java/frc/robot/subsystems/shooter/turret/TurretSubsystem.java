package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX m_turretMotor;
    private final SmartMotorControllerConfig m_motorConfig;
    private final SmartMotorController m_motor;
    private final Pivot m_turret;

    public TurretSubsystem() {
        m_turretMotor = new TalonFX(TurretConstants.CAN_ID);
        m_motorConfig = TurretConstants.SMC_CONFIG.withSubsystem(this);
        m_motor = new TalonFXWrapper(m_turretMotor, TurretConstants.MOTOR, m_motorConfig);
        m_turret = new Pivot(TurretConstants.PIVOT_CONFIG, m_motor);
    }

    public Pose2d getPose(Pose2d robotPose) {
        return robotPose.plus(new Transform2d(
                TurretConstants.ROBOT_TO_TURRET.getTranslation().toTranslation2d(),
                TurretConstants.ROBOT_TO_TURRET.getRotation().toRotation2d()));
    }

    public ChassisSpeeds getVelocity(ChassisSpeeds robotVelocity, Angle robotAngle) {
        Translation2d rRobot = TurretConstants.ROBOT_TO_TURRET.getTranslation().toTranslation2d(); // in robot frame
        Translation2d rWorld = rRobot.rotateBy(Rotation2d.fromRadians(robotAngle.in(Radians))); // rotate into field
                                                                                                // frame

        double omega = robotVelocity.omegaRadiansPerSecond; // robot yaw rate (rad/s)

        // rotational linear velocity at turret (v_rot = ω × r_world)
        double vRotX = -omega * rWorld.getY();
        double vRotY = omega * rWorld.getX();

        // final turret linear velocity in field frame
        double turretVx = robotVelocity.vxMetersPerSecond + vRotX;
        double turretVy = robotVelocity.vyMetersPerSecond + vRotY;

        // turret angular velocity in field frame
        double turretOmega = omega + m_motor.getMechanismVelocity().in(RadiansPerSecond);

        return new ChassisSpeeds(turretVx, turretVy, turretOmega);
    }

    public void periodic() {
        m_turret.updateTelemetry();
    }

    public void simulationPeriodic() {
        m_turret.simIterate();
    }

    public Command turretCmd(double dutycycle) {
        return m_turret.set(dutycycle);
    }

    public Command setAngle(Angle angle) {
        return m_turret.setAngle(angle);
    }

    public void setAngleSetpoint(Angle measure) {
        m_turret.setMechanismPositionSetpoint(measure);
    }
}

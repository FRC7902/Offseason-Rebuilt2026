package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.telemetry.SmartMotorControllerTelemetryConfig;

public class TurretConstants {

    public static final double[] GEAR_RATIO = { 144 / 15, 5, 1.08 };
    public static final DCMotor MOTOR = DCMotor.getKrakenX60(1);
    public static final int CAN_ID = 8;

    public static final Angle TOLERANCE = Degrees.of(1);

    public static final SmartMotorControllerConfig SMC_CONFIG = new SmartMotorControllerConfig()
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withSimClosedLoopController(0.0, 0.0, 0)
            .withClosedLoopController(0.0, 0.0, 0)
            .withGearing(new MechanismGearing(new GearBox(GEAR_RATIO)))
            .withIdleMode(MotorMode.BRAKE)
            .withMotorInverted(false)
            .withFeedforward(new ArmFeedforward(0.5, 0.0, 5.0, 0))
            .withSimFeedforward(new ArmFeedforward(0.5, 0.0, 5.0, 0))
            .withTelemetry("TurretMotor", new SmartMotorControllerTelemetryConfig()
                    .withTelemetryVerbosity(SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
                    .withDataLogName("TurretMotor")
                    .withNetworkTables(!DriverStation.isFMSAttached()))
            .withStatorCurrentLimit(Amps.of(60))
            .withStartingPosition(Degrees.of(0))
            .withMomentOfInertia(yams.units.YUnits.PoundSquareInches.of(0.01));

    public static final PivotConfig PIVOT_CONFIG = new PivotConfig()
            .withHardLimits(Degrees.of(-360), Degrees.of(360))
            .withTelemetry("TurretMech", TelemetryVerbosity.HIGH);

    public static final Transform3d ROBOT_TO_TURRET = new Transform3d(
            Feet.of(-1.5), Feet.of(0), Feet.of(0.5), Rotation3d.kZero);
}

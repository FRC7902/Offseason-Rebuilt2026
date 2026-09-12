package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.telemetry.SmartMotorControllerTelemetryConfig;

public class TurretConstants {

  public static final DCMotor MOTOR = DCMotor.getKrakenX44Foc(1);
  public static final int CAN_ID = 42;

  public static final Angle MIN_ANGLE = Degrees.of(-160);
  public static final Angle MAX_ANGLE = Degrees.of(190);
  public static final Angle CCW_90_ANGLE = Degrees.of(90);
  public static final Angle CW_90_ANGLE = Degrees.of(-90);
  public static final Angle ANGLE_180 = Degrees.of(180);

  public static final Angle TOLERANCE = Degrees.of(1);
  public static final Angle RIGHT_ANGLE_TOLERANCE = Degrees.of(45);

  public static final SmartMotorControllerConfig SMC_CONFIG =
      new SmartMotorControllerConfig()
          .withClosedLoopController(150.0, 0.0, 1)
          .withSimClosedLoopController(42.65, 0.0, 3)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(45)))
          .withIdleMode(MotorMode.BRAKE)
          .withMotorInverted(false)
          .withFeedforward(new SimpleMotorFeedforward(0.042757, 5.5173, 0.83544, 0.02))
          .withSimFeedforward(new SimpleMotorFeedforward(0.01025, 0.0, 0.0, 0.02))
          .withTelemetry(
              "TurretMotor",
              new SmartMotorControllerTelemetryConfig()
                  .withTelemetryVerbosity(SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
                  .withDataLogName("TurretMotor")
                  .withNetworkTables(!DriverStation.isFMSAttached()))
          .withStatorCurrentLimit(Amps.of(40))
          .withSupplyCurrentLimit(Amps.of(40))
          .withSoftLimits(MIN_ANGLE, MAX_ANGLE)
          .withSimStartingPosition(Degrees.zero())
          .withMomentOfInertia(yams.units.YUnits.PoundSquareInches.of(0.01)); // TODO

  public static final PivotConfig PIVOT_CONFIG =
      new PivotConfig()
          .withHardLimits(TurretConstants.MIN_ANGLE, TurretConstants.MAX_ANGLE)
          .withTelemetry("TurretMech", TelemetryVerbosity.HIGH);

  public static final Transform3d ROBOT_TO_TURRET =
      new Transform3d(Feet.of(-1.5), Feet.of(0), Feet.of(0.5), Rotation3d.kZero); // TODO
}

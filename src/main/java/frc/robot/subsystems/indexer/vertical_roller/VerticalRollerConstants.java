package frc.robot.subsystems.indexer.vertical_roller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.telemetry.SmartMotorControllerTelemetryConfig;

public class VerticalRollerConstants {
  public static final DCMotor MOTOR = DCMotor.getNEO(1);
  public static final int CAN_ID = 34;

  public static final AngularVelocity TOLERANCE = RPM.of(10); // TODO

  public static final AngularVelocity FEEDING_SPEED = RPM.of(750); // TODO
  public static final AngularVelocity STORING_SPEED = RPM.of(200); // TODO

  public static final double FEEDING_DUTY_CYCLE = 1.0;
  public static final double STORING_DUTY_CYCLE = 0.2;

  public static final SmartMotorControllerConfig SMC_CONFIG =
      new SmartMotorControllerConfig()
          .withClosedLoopController(0.00016541, 0, 0) // TODO
          .withSimClosedLoopController(0, 0, 0)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(2.18)))
          .withIdleMode(MotorMode.COAST)
          .withTelemetry(
              "VerticalRollerMotor",
              new SmartMotorControllerTelemetryConfig()
                  .withTelemetryVerbosity(SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
                  // .withDataLogName("VerticalRollerMotor")
                  .withNetworkTables(!DriverStation.isFMSAttached()))
          .withStatorCurrentLimit(Amps.of(40)) // TODO
          .withMotorInverted(false)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557)) // TODO
          .withSimFeedforward(new SimpleMotorFeedforward(0.0102, 0.4382, 0));

  public static final FlyWheelConfig FLY_WHEEL_CONFIG =
      new FlyWheelConfig()
          .withDiameter(Inches.of(1.25))
          .withTelemetry("VerticalRollerMech", TelemetryVerbosity.HIGH)
          .withSpeedometerSimulation(RPM.of(1619));
}

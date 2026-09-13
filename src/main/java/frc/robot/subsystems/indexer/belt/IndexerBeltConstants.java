package frc.robot.subsystems.indexer.belt;

import static edu.wpi.first.units.Units.Amps;
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

public class IndexerBeltConstants {

  public static final DCMotor LEADER_MOTOR = DCMotor.getKrakenX44Foc(1);
  public static final DCMotor FOLLOWER_MOTOR = DCMotor.getKrakenX44Foc(1);
  public static final int LEADER_CAN_ID = 1;
  public static final int FOLLOWER_CAN_ID = 53;

  public static final AngularVelocity TOLERANCE = RPM.of(10); // TODO

  public static final AngularVelocity FEEDING_SPEED = RPM.of(750); // TODO
  public static final AngularVelocity STORING_SPEED = RPM.of(100); // TODO

  public static final double FEEDING_DUTY_CYCLE = 1.0;
  public static final double STORING_DUTY_CYCLE = 0.2;

  public static final SmartMotorControllerConfig LEADER_SMC_CONFIG =
      new SmartMotorControllerConfig()
          .withClosedLoopController(0.0, 0, 0) // TODO
          .withSimClosedLoopController(0, 0, 0)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(2.1818)))
          .withIdleMode(MotorMode.COAST)
          .withTelemetry(
              "IndexerBeltMotor",
              new SmartMotorControllerTelemetryConfig()
                  .withTelemetryVerbosity(SmartMotorControllerConfig.TelemetryVerbosity.LOW)
                  .withDataLogName("IndexerBeltMotor")
                  .withNetworkTables(!DriverStation.isFMSAttached()))
          .withStatorCurrentLimit(Amps.of(40)) // TODO
          .withSupplyCurrentLimit(Amps.of(40)) // TODO
          .withMotorInverted(false)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new SimpleMotorFeedforward(0.35, 0.217, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0.0102, 0.21, 0));

  public static final SmartMotorControllerConfig FOLLOWER_SMC_CONFIG =
      LEADER_SMC_CONFIG.clone().withMotorInverted(true); // TODO

  public static final FlyWheelConfig FLY_WHEEL_CONFIG =
      new FlyWheelConfig()
          .withTelemetry("IndexerBeltMech", TelemetryVerbosity.LOW)
          .withSpeedometerSimulation(RPM.of(3377));
}

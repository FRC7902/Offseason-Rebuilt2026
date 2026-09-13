package frc.robot.subsystems.indexer.roller_floor;

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

public class RollerFloorConstants {

  public static final DCMotor MOTOR = DCMotor.getKrakenX60Foc(1);
  public static final int CAN_ID = 52;

  public static final AngularVelocity TOLERANCE = RPM.of(10); // TODO

  private static final AngularVelocity MAX_SPEED_RPM =
      RPM.of(1619); // Theoretical max speed of the motor with gearing applied

  public static final AngularVelocity FEEDING_SPEED = MAX_SPEED_RPM.times(0.60); // 60% of max speed
  public static final AngularVelocity STORING_SPEED = MAX_SPEED_RPM.times(0.20); // 20% of max speed

  public static final AngularVelocity IS_REVERSING_SPEED_MIN = RPM.of(-500);

  public static final SmartMotorControllerConfig SMC_CONFIG =
      new SmartMotorControllerConfig()
          .withClosedLoopController(0, 0, 0) // TODO
          .withSimClosedLoopController(0, 0, 0)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3.57142857143)))
          .withIdleMode(MotorMode.COAST)
          .withTelemetry(
              "RollerFloorMotor",
              new SmartMotorControllerTelemetryConfig()
                  .withTelemetryVerbosity(SmartMotorControllerConfig.TelemetryVerbosity.LOW)
                  .withDataLogName("RollerFloorMotor")
                  .withNetworkTables(!DriverStation.isFMSAttached()))
          .withStatorCurrentLimit(Amps.of(40)) // TODO
          .withSupplyCurrentLimit(Amps.of(40)) // TODO
          .withMotorInverted(true)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new SimpleMotorFeedforward(0.35, 0.43, 0.0))
          .withSimFeedforward(new SimpleMotorFeedforward(0.0102, 0.4382, 0));

  public static final FlyWheelConfig FLY_WHEEL_CONFIG =
      new FlyWheelConfig()
          .withDiameter(Inches.of(1.25))
          .withTelemetry("RollerFloorMech", TelemetryVerbosity.LOW)
          .withSpeedometerSimulation(MAX_SPEED_RPM);
}

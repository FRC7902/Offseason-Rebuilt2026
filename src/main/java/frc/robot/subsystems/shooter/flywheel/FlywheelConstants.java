package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
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

public class FlywheelConstants {

  public static final DCMotor LEADER_MOTOR = DCMotor.getKrakenX60Foc(1); // TODO
  public static final DCMotor FOLLOWER_MOTOR = DCMotor.getKrakenX60Foc(1); // TODO
  public static final int LEADER_CAN_ID = 6; // TODO
  public static final int FOLLOWER_CAN_ID = 10; // TODO

  public static final AngularVelocity TOLERANCE = RPM.of(10); // TODO

  public static final SmartMotorControllerConfig LEADER_SMC_CONFIG =
      new SmartMotorControllerConfig()
          .withClosedLoopController(0.00016541, 0, 0) // TODO
          .withSimClosedLoopController(0.00016541, 0, 0) // TODO
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
          .withIdleMode(MotorMode.COAST) // TODO
          .withTelemetry(
              "ShooterFlywheelMotor",
              new SmartMotorControllerTelemetryConfig()
                  .withTelemetryVerbosity(SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
                  .withDataLogName("ShooterFlywheelMotor")
                  .withNetworkTables(!DriverStation.isFMSAttached()))
          .withStatorCurrentLimit(Amps.of(40)) // TODO
          .withMotorInverted(false) // TODO
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557)) // TODO
          .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557)) // TODO
          .withMomentOfInertia(Inches.of(4), Pounds.of(1)); // TODO

  public static final SmartMotorControllerConfig FOLLOWER_SMC_CONFIG =
      LEADER_SMC_CONFIG.clone().withMotorInverted(true);

  public static final FlyWheelConfig FLY_WHEEL_CONFIG =
      new FlyWheelConfig()
          .withDiameter(Inches.of(4)) 
          .withTelemetry("ShooterFlywheelMech", TelemetryVerbosity.HIGH)
          .withSpeedometerSimulation(RPM.of(750)); // TODO
}

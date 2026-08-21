package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.telemetry.SmartMotorControllerTelemetryConfig;

public class HoodConstants {

  public static final DCMotor MOTOR = DCMotor.getKrakenX60Foc(1); // TODO
  public static final int CAN_ID = 7; // TODO

  public static final Angle TOLERANCE = Degrees.of(1); // TODO

  public static final Angle MIN_ANGLE = Degrees.of(-30); // TODO
  public static final Angle MAX_ANGLE = Degrees.of(100); // TODO

  public static final SmartMotorControllerConfig SMC_CONFIG =
      new SmartMotorControllerConfig()
          .withClosedLoopController(4, 0, 0) // TODO
          .withSoftLimits(MIN_ANGLE, MAX_ANGLE) // TODO
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4))) // TODO
          .withIdleMode(MotorMode.BRAKE) // TODO
          .withTelemetry(
              "HoodMotor",
              new SmartMotorControllerTelemetryConfig()
                  .withTelemetryVerbosity(SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
                  .withDataLogName("HoodMotor")
                  .withNetworkTables(!DriverStation.isFMSAttached()))
          .withStatorCurrentLimit(Amps.of(40)) // TODO
          .withMotorInverted(false) // TODO
          .withClosedLoopRampRate(Seconds.of(0.25)) // TODO
          .withFeedforward(new ArmFeedforward(0, 0, 0, 0)) // TODO
          .withSimStartingPosition(Degrees.of(0)); // TODO

  public static final ArmConfig ARM_CONFIG =
      new ArmConfig()
          .withLength(Meters.of(0.135)) // TODO
          .withHardLimits(MIN_ANGLE, MAX_ANGLE) // TODO
          .withTelemetry("HoodMech", TelemetryVerbosity.HIGH); // TODO
}

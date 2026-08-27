package frc.robot.subsystems.intake.linear;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.telemetry.SmartMotorControllerTelemetryConfig;

public class LinearIntakeConstants {

  public static final DCMotor MOTOR = DCMotor.getFalcon500(1); // TODO
  public static final int CAN_ID = 4; // TODO

  public static final Distance TOLERANCE = Meters.of(0.1); // TODO

  public static final Distance FULLY_RETRACTED = Meters.of(0); // TODO
  public static final Distance FULLY_EXTENDED = Meters.of(2); // TODO

  // TODO: Add distance constant for midpoint (intake extended as far as possible without letting
  // fuel loose)
  // TODO: Add distance constants for positions to shuffle between

  public static final SmartMotorControllerConfig SMC_CONFIG =
      new SmartMotorControllerConfig()
          .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22)) // TODO
          .withClosedLoopController(4, 0, 0) // TODO
          .withTrapezoidalProfile(MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5)) // TODO
          .withSoftLimits(FULLY_RETRACTED, FULLY_EXTENDED)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(1))) // TODO
          .withIdleMode(MotorMode.BRAKE) // TODO
          .withTelemetry(
              "LinearIntakeMotor",
              new SmartMotorControllerTelemetryConfig()
                  .withTelemetryVerbosity(SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
                  .withDataLogName("LinearIntakeMotor")
                  .withNetworkTables(!DriverStation.isFMSAttached()))
          .withStatorCurrentLimit(Amps.of(40)) // TODO
          .withMotorInverted(false) // TODO
          .withFeedforward(new ElevatorFeedforward(0, 0, 0, 0)) // TODO
          .withStartingPosition(Meters.of(0.5)); // TODO

  private static final MechanismPositionConfig ROBOT_TO_MECHANISM =
      new MechanismPositionConfig()
          .withMaxRobotHeight(Meters.of(1.5)) // TODO
          .withMaxRobotLength(Meters.of(0.75)) // TODO
          .withRelativePosition(
              new Translation3d(Meters.of(-0.25), Meters.of(0), Meters.of(0.5))); // TODO

  public static final ElevatorConfig ELEVATOR_CONFIG =
      new ElevatorConfig()
          .withHardLimits(FULLY_RETRACTED, FULLY_EXTENDED)
          .withTelemetry("LinearIntakeMech", TelemetryVerbosity.HIGH)
          .withMechanismPositionConfig(ROBOT_TO_MECHANISM)
          .withCarriageWeight(Pounds.of(16)); // TODO
}

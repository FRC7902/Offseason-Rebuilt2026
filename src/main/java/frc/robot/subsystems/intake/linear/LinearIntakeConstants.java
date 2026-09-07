package frc.robot.subsystems.intake.linear;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
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

  public static final DCMotor MOTOR = DCMotor.getKrakenX60Foc(1);
  public static final int CAN_ID = 20;

  public static final Distance TOLERANCE = Meters.of(0.05);

  public static final Distance FULLY_RETRACTED = Meters.zero();
  public static final Distance FULLY_EXTENDED = Meters.of(0.258915);
  public static final Distance MIDPOINT_DISTANCE = Meters.of(0.2);

  public static final Distance NEAR_FULLY_RETRACTED = Meters.of(0.1);
  public static final Distance SHUFFLE_MIDPOINT = Meters.of(0.15);

  public static final Distance[] FIRST_SHUFFLE_DISTANCES = {
    SHUFFLE_MIDPOINT, FULLY_EXTENDED, MIDPOINT_DISTANCE, SHUFFLE_MIDPOINT, NEAR_FULLY_RETRACTED
  };

  public static final Distance[] SECOND_SHUFFLE_DISTANCES = {
    NEAR_FULLY_RETRACTED,
    MIDPOINT_DISTANCE,
    NEAR_FULLY_RETRACTED,
    MIDPOINT_DISTANCE,
    FULLY_RETRACTED
  };

  public static final Distance[] REPEATING_SHUFFLE_DISTANCES = {
    FULLY_RETRACTED, NEAR_FULLY_RETRACTED
  };

  public static final SmartMotorControllerConfig SMC_CONFIG =
      new SmartMotorControllerConfig()
          .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 10))
          .withClosedLoopController(5, 0, 0)
          .withSimClosedLoopController(25, 0, 0.3)
          .withTrapezoidalProfile(MetersPerSecond.of(1.5), MetersPerSecondPerSecond.of(1.5))
          .withSoftLimits(FULLY_RETRACTED, FULLY_EXTENDED)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(5.0625)))
          .withIdleMode(MotorMode.COAST)
          .withTelemetry(
              "LinearIntakeMotor",
              new SmartMotorControllerTelemetryConfig()
                  .withTelemetryVerbosity(SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
                  .withDataLogName("LinearIntakeMotor")
                  .withNetworkTables(!DriverStation.isFMSAttached()))
          .withStatorCurrentLimit(Amps.of(40)) // TODO
          .withSupplyCurrentLimit(Amps.of(10)) // TODO
          .withMotorInverted(false)
          .withFeedforward(new ElevatorFeedforward(0.28544, -0.047288, 0.64354, 0.028718))
          .withSimFeedforward(new ElevatorFeedforward(0, 0.1998, 0, 0))
          .withSimStartingPosition(FULLY_RETRACTED);

  private static final MechanismPositionConfig ROBOT_TO_MECHANISM =
      new MechanismPositionConfig()
          .withMaxRobotHeight(Meters.of(1.5))
          .withMaxRobotLength(Meters.of(0.75))
          .withRelativePosition(new Translation3d(Meters.of(-0.25), Meters.of(0), Meters.of(0.5)));

  public static final Angle MECHANISM_ANGLE = Degrees.of(15.626606);

  public static final ElevatorConfig ELEVATOR_CONFIG =
      new ElevatorConfig()
          .withHardLimits(FULLY_RETRACTED, FULLY_EXTENDED)
          .withTelemetry("LinearIntakeMech", TelemetryVerbosity.HIGH)
          .withMechanismPositionConfig(ROBOT_TO_MECHANISM)
          .withAngle(MECHANISM_ANGLE)
          .withCarriageWeight(Pounds.of(7.933));
}

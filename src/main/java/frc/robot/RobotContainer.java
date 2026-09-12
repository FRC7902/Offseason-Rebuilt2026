// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.indexer.IndexerSystem;
import frc.robot.subsystems.indexer.belt.IndexerBeltSubsystem;
import frc.robot.subsystems.indexer.feeder.FeederSubsystem;
import frc.robot.subsystems.indexer.roller_floor.RollerFloorSubsystem;
import frc.robot.subsystems.indexer.vertical_roller.VerticalRollerSubsystem;
import frc.robot.subsystems.intake.IntakeSystem;
import frc.robot.subsystems.intake.linear.LinearIntakeSubsystem;
import frc.robot.subsystems.intake.roller.IntakeRollerSubsystem;
import frc.robot.subsystems.shooter.ShooterSystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveConstants;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer {

  private final CommandPS5Controller m_driverController;

  private final IndexerBeltSubsystem m_indexerBeltSubsystem;
  private final FeederSubsystem m_feederSubsystem;
  private final RollerFloorSubsystem m_rollerFloorSubsystem;
  private final VerticalRollerSubsystem m_verticalRollerSubsystem;

  private final LinearIntakeSubsystem m_linearIntakeSubsystem;
  private final IntakeRollerSubsystem m_intakeRollerSubsystem;

  private final FlywheelSubsystem m_flywheelSubsystem;
  private final HoodSubsystem m_hoodSubsystem;
  private final TurretSubsystem m_turretSubsystem;

  private final IndexerSystem m_indexerSystem;
  private final IntakeSystem m_intakeSystem;
  private final ShooterSystem m_shooterSystem;

  private final SwerveDriveSubsystem m_swerveDriveSubsystem;

  private double applyDriverTranslationStickCurve(double input) {
    return Math.copySign(
        Math.pow(Math.abs(input), SwerveDriveConstants.DRIVER_TRANSLATION_STICK_CURVE_EXPONENT),
        input);
  }

  private double getCurvedDriverLeftX() {
    return applyDriverTranslationStickCurve(-m_driverController.getLeftX());
  }

  private double getCurvedDriverLeftY() {
    return applyDriverTranslationStickCurve(-m_driverController.getLeftY());
  }

  SwerveInputStream driveAngularVelocity;
  SwerveInputStream driveSlowAngularVelocity;
  SwerveInputStream driveDirectAngle;

  Command driveFieldOrientedAngularVelocity;
  Command driveSlowFieldOrientedAngularVelocity;
  Command driveFieldOrientedDirectAngle;

  private final StructArrayPublisher<Pose3d> posesPublisher;

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    m_driverController = new CommandPS5Controller(Constants.DRIVER_CONTROLLER_PORT);

    /*
     * Swerve drive subsystem and input streams
     */
    m_swerveDriveSubsystem =
        new SwerveDriveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    driveAngularVelocity =
        SwerveInputStream.of(
                m_swerveDriveSubsystem.getSwerveDrive(),
                this::getCurvedDriverLeftY,
                this::getCurvedDriverLeftX)
            .withControllerRotationAxis(() -> m_driverController.getRightX() * -1)
            .deadband(Constants.DRIVER_CONTROLLER_DEADBAND)
            .scaleTranslation(1.0)
            .allianceRelativeControl(true);
    driveSlowAngularVelocity =
        driveAngularVelocity.copy().scaleTranslation(SwerveDriveConstants.SLOW_MODE_SCALE);
    driveDirectAngle =
        driveAngularVelocity
            .copy()
            .withControllerHeadingAxis(m_driverController::getRightX, m_driverController::getRightY)
            .headingWhile(true);
    driveFieldOrientedAngularVelocity =
        m_swerveDriveSubsystem.driveFieldOriented(driveAngularVelocity);
    driveSlowFieldOrientedAngularVelocity =
        m_swerveDriveSubsystem.driveFieldOriented(driveSlowAngularVelocity);
    driveFieldOrientedDirectAngle = m_swerveDriveSubsystem.driveFieldOriented(driveDirectAngle);

    // Publish the poses of the components to NetworkTables for visualization in 3D
    posesPublisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("/3D/ComponentPoses", Pose3d.struct)
            .publish();

    // TODO: Enable data logging once USB stick is connected
    // Start data logging
    // DataLogManager.start();
    // Include DriverStation data in the log
    // DriverStation.startDataLog(DataLogManager.getLog());

    m_indexerBeltSubsystem = new IndexerBeltSubsystem();
    m_feederSubsystem = new FeederSubsystem();
    m_rollerFloorSubsystem = new RollerFloorSubsystem();
    m_verticalRollerSubsystem = new VerticalRollerSubsystem();

    m_linearIntakeSubsystem = new LinearIntakeSubsystem();
    m_intakeRollerSubsystem = new IntakeRollerSubsystem();

    m_flywheelSubsystem = new FlywheelSubsystem();
    m_hoodSubsystem = new HoodSubsystem();
    m_turretSubsystem = new TurretSubsystem();

    m_indexerSystem =
        new IndexerSystem(
            m_indexerBeltSubsystem,
            m_feederSubsystem,
            m_rollerFloorSubsystem,
            m_verticalRollerSubsystem);
    m_intakeSystem = new IntakeSystem(m_linearIntakeSubsystem, m_intakeRollerSubsystem);
    m_shooterSystem = new ShooterSystem(m_flywheelSubsystem, m_hoodSubsystem, m_turretSubsystem);

    // NamedCommands.registerCommand("extendAndIntake",
    // m_intakeSystem.extendAndIntake());

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    configureBindings();
  }

  private void configureBindings() {

    m_swerveDriveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);

    Trigger intakeTrigger = m_driverController.L2();
    Trigger shootTrigger = m_driverController.R2();

    m_driverController
        .options()
        .onTrue((Commands.runOnce(m_swerveDriveSubsystem::zeroGyroWithAlliance)));
    m_driverController.create().whileTrue(m_swerveDriveSubsystem.centerModulesCommand());

    // Neither intake button nor shoot button is pressed
    intakeTrigger
        .negate()
        .and(shootTrigger.negate())
        .onTrue(m_intakeSystem.stop()) // Stop intaking
        .onTrue(m_shooterSystem.stopShooting()) // Stop shooting
        .onTrue(m_indexerSystem.stop()); // Stop indexing

    // Intake button is pressed, but shoot button is not pressed
    intakeTrigger
        .negate()
        .and(shootTrigger)
        .onTrue(m_intakeSystem.shuffle()) // Shuffle hopper
        .onTrue(m_shooterSystem.aimAndShoot()) // Aim and shoot
        .onTrue(m_indexerSystem.feedFuel()); // Feed fuel to shooter

    // Shoot button is pressed, but intake button is not pressed
    intakeTrigger
        .and(shootTrigger.negate())
        .onTrue(m_intakeSystem.extendAndIntake()) // Extend and intake
        .onTrue(m_shooterSystem.stopShooting()) // Stop shooting
        .onTrue(m_indexerSystem.storeFuel()); // Funnel fuel inside indexer

    // Both intake button and shoot button are pressed
    intakeTrigger
        .and(shootTrigger)
        .onTrue(m_intakeSystem.extendAndIntake()) // Extend and intake
        .onTrue(m_shooterSystem.aimAndShoot()) // Aim and shoot
        .onTrue(m_indexerSystem.feedFuel()); // Feed fuel to shooter
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void publishComponentPoses() {
    Pose3d turretPose = m_turretSubsystem.getPose3d();
    Pose3d hoodPose = m_hoodSubsystem.getPose3d(turretPose);
    Pose3d linearIntakePose = m_linearIntakeSubsystem.getPose3d();

    posesPublisher.set(new Pose3d[] {linearIntakePose, turretPose, hoodPose, new Pose3d()});
  }

  public void calibrateLinearIntakePosition() {
    boolean leftExtended = m_linearIntakeSubsystem.getLeftExtendedLimitSwitch();
    boolean rightExtended = m_linearIntakeSubsystem.getRightExtendedLimitSwitch();
    boolean leftRetracted = m_linearIntakeSubsystem.getLeftRetractedLimitSwitch();
    boolean rightRetracted = m_linearIntakeSubsystem.getRightRetractedLimitSwitch();

    if (leftExtended || rightExtended) {
      m_linearIntakeSubsystem.setEncoderPositionExtended();
    } else if (leftRetracted || rightRetracted) {
      m_linearIntakeSubsystem.setEncoderPositionRetracted();
    }
  }
}

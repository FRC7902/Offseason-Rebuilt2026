// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
<<<<<<< HEAD
import com.pathplanner.lib.auto.NamedCommands;
=======
>>>>>>> 6f73fbd (Added new auto with mirror counterpart)
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
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
import swervelib.SwerveInputStream;

public class RobotContainer {

  private final CommandPS5Controller m_driverController;

  private final IndexerSystem m_indexerSystem;
  private final IntakeSystem m_intakeSystem;
  public final ShooterSystem m_shooterSystem;

  private final LinearIntakeSubsystem m_linearIntakeSubsystem;
  private final HoodSubsystem m_hoodSubsystem;
  private final TurretSubsystem m_turretSubsystem;

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
    m_swerveDriveSubsystem = SwerveDriveSubsystem.getInstance();
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
        driveAngularVelocity
            .copy()
            .scaleTranslation(SwerveDriveConstants.SLOW_MODE_TRANSLATION_SCALE)
            .scaleRotation(SwerveDriveConstants.SLOW_MODE_ROTATION_SCALE);
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

    m_linearIntakeSubsystem = LinearIntakeSubsystem.getInstance();
    m_hoodSubsystem = HoodSubsystem.getInstance();
    m_turretSubsystem = TurretSubsystem.getInstance();

    m_indexerSystem =
        new IndexerSystem(
            IndexerBeltSubsystem.getInstance(),
            FeederSubsystem.getInstance(),
            RollerFloorSubsystem.getInstance(),
            VerticalRollerSubsystem.getInstance());
    m_intakeSystem =
        new IntakeSystem(LinearIntakeSubsystem.getInstance(), IntakeRollerSubsystem.getInstance());
    m_shooterSystem =
        new ShooterSystem(
            FlywheelSubsystem.getInstance(),
            HoodSubsystem.getInstance(),
            TurretSubsystem.getInstance());

    NamedCommands.registerCommand("extendAndIntake", m_intakeSystem.extendAndIntake());
    NamedCommands.registerCommand("stopIntake", m_intakeSystem.stop());
    NamedCommands.registerCommand("feedFuel", m_indexerSystem.feedFuel());
    NamedCommands.registerCommand("storeFuel", m_indexerSystem.storeFuel());
    NamedCommands.registerCommand("aimAndShoot", m_shooterSystem.aimAndShoot());
<<<<<<< HEAD
<<<<<<< HEAD
    NamedCommands.registerCommand("extendAndIntake", m_intakeSystem.extendAndIntake());
    NamedCommands.registerCommand("stopIntake", m_intakeSystem.stop());
    NamedCommands.registerCommand("feedFuel", m_indexerSystem.feedFuel());
    NamedCommands.registerCommand("storeFuel", m_indexerSystem.storeFuel());
    NamedCommands.registerCommand("aimAndShoot", m_shooterSystem.aimAndShoot());
    NamedCommands.registerCommand("stopShooter", m_shooterSystem.stop());
=======
>>>>>>> 6d6b64e (FIxed issues with mirror)
=======
    NamedCommands.registerCommand("stopShooter", m_shooterSystem.stop());
>>>>>>> 6f73fbd (Added new auto with mirror counterpart)

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    configureBindings();
  }

  public Command setFlywheelDefaultSpeed() {
    return m_shooterSystem.flywheelDefaultSpeed();
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
        .onTrue(m_shooterSystem.stop()) // Stop shooting
        .onTrue(m_indexerSystem.stop()); // Stop indexing

    // Shoot button is pressed, but intake button is not pressed
    intakeTrigger
        .negate()
        .and(shootTrigger)
        .onTrue(m_shooterSystem.aimAndShoot()) // Aim and shoot
        .onTrue(
            Commands.sequence(
                Commands.waitUntil(m_shooterSystem::isShooterReady),
                m_indexerSystem.reverseIndexer().withTimeout(0.5),
                Commands.parallel(
                    m_intakeSystem.shuffle(), // Shuffle hopper
                    m_indexerSystem.feedFuel() // Feed fuel to shooter
                    )))
        .whileTrue(driveSlowFieldOrientedAngularVelocity);

    // Intake button is pressed, but shoot button is not pressed
    intakeTrigger
        .and(shootTrigger.negate())
        .onTrue(m_intakeSystem.extendAndIntake()) // Extend and intake
        .onTrue(m_shooterSystem.stop()) // Stop shooting
        .onTrue(m_indexerSystem.storeFuel()); // Funnel fuel inside indexer

    // Both intake button and shoot button are pressed
    intakeTrigger
        .and(shootTrigger)
        .onTrue(m_intakeSystem.extendAndIntake()) // Extend and intake
        // .onTrue(m_shooterSystem.aimAndShoot()) // Aim and shoot
        .onTrue(m_indexerSystem.feedFuel()) // Feed fuel to shooter
        .whileTrue(driveSlowFieldOrientedAngularVelocity);

    // Manual Shoot Button Bindings
    Trigger manualShootTrigger = m_driverController.R1();

    // Neither intake button nor manual shoot button is pressed
    intakeTrigger
        .negate()
        .and(manualShootTrigger.negate())
        .onTrue(m_intakeSystem.stop()) // Stop intaking
        .onTrue(m_shooterSystem.stop()) // Stop shooting
        .onTrue(m_indexerSystem.stop()); // Stop indexing

    // manual Shoot button is pressed, but intake button is not pressed
    intakeTrigger
        .negate()
        .and(manualShootTrigger)
        .onTrue(m_shooterSystem.manualAimAndShoot()) // Aim and shoot
        .onTrue(
            m_indexerSystem
                .reverseIndexer()
                .withTimeout(0.5)
                .andThen(
                    Commands.parallel(
                        m_intakeSystem.shuffle(), // Shuffle hopper
                        m_indexerSystem.feedFuel() // Feed fuel to shooter
                        )))
        .whileTrue(driveSlowFieldOrientedAngularVelocity);

    // Intake button is pressed, but manual shoot button is not pressed
    intakeTrigger
        .and(manualShootTrigger.negate())
        .onTrue(m_intakeSystem.extendAndIntake()) // Extend and intake
        .onTrue(m_shooterSystem.stop()) // Stop shooting
        .onTrue(m_indexerSystem.storeFuel()); // Funnel fuel inside indexer

    // Both intake button and manual shoot button are pressed
    intakeTrigger
        .and(manualShootTrigger)
        .onTrue(m_intakeSystem.extendAndIntake()) // Extend and intake
        .onTrue(m_shooterSystem.manualAimAndShoot()) // Aim and shoot
        .onTrue(m_indexerSystem.feedFuel()) // Feed fuel to shooter
        .whileTrue(driveSlowFieldOrientedAngularVelocity);

    Trigger outtakeTrigger = m_driverController.L1();

    outtakeTrigger
        .onTrue(m_intakeSystem.extendAndOuttake())
        .onTrue(m_indexerSystem.reverseIndexer());

    outtakeTrigger.onFalse(m_intakeSystem.stop()).onFalse(m_indexerSystem.stop());
    /*
     * Manual driving for swerve tuning
     */

    m_driverController.povUp().whileTrue(m_swerveDriveSubsystem.driveForward());
    m_driverController.povDown().whileTrue(m_swerveDriveSubsystem.driveBackward());
    m_driverController.povLeft().whileTrue(m_swerveDriveSubsystem.driveLeft());
    m_driverController.povRight().whileTrue(m_swerveDriveSubsystem.driveRight());
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

  public Command stopAllSubsystems() {
    return Commands.parallel(m_intakeSystem.stop(), m_shooterSystem.stop(), m_indexerSystem.stop());
  }
}

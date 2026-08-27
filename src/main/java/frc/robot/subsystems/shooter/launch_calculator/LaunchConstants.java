package frc.robot.subsystems.shooter.launch_calculator;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class LaunchConstants {
    public static final double towerPresetDistance = 2.5;
    public static final double trenchPresetDistance = 3.03;
    public static final double outpostPresetDistance = 4.84;
    public static final double passingPresetDistance = 5.5;
    public static final double xPassTarget = Units.inchesToMeters(37);
    public static final double yPassTarget = Units.inchesToMeters(65);

    //TODO: Fill in robotToLauncher
    public static final Transform3d robotToLauncher = new Transform3d();
}

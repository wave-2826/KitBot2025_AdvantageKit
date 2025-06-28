package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout
        .loadField(AprilTagFields.k2025ReefscapeWelded);

    // Camera names, must match names configured on coprocessor
    public static String leftCameraHostname = "limelight-ps";
    public static String rightCameraHostname = "limelight-reef";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    // Using limelights but for sim we are emulating PhotonVision
    public static Transform3d leftCameraPos = new Transform3d(Units.inchesToMeters(11.1),
        Units.inchesToMeters(11.459224), Units.inchesToMeters(32.398761),
        new Rotation3d(Math.toRadians(0), Math.toRadians(-30), Math.toRadians(180)));

    public static Transform3d rightCameraPos = new Transform3d(Units.inchesToMeters(-5),
        Units.inchesToMeters(-14.143929), Units.inchesToMeters(14.175),
        new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(0)));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 1000000000; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
    };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
}

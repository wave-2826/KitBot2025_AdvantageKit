package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout
        .loadField(AprilTagFields.k2025ReefscapeWelded);

    // Camera names, must match names configured on coprocessor
    public static String leftCameraHostname = "limelight-left";
    public static String rightCameraHostname = "limelight-right";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    // Using limelights but for sim we are emulating PhotonVision
    public static Transform3d leftCameraPos = new Transform3d(
        new Translation3d(0.23840027576590855, 0.21439095933876529, 0.34860957146726285),
        new Rotation3d(-0.04998098341920379, 1.0067494270931172, -0.8353690259370719));

    public static Transform3d rightCameraPos = new Transform3d(
        new Translation3d(0.24873265280001045, -0.21439095933876529, 0.3606904011261275),
        new Rotation3d(0.02239841926409702, 0.04025359873626376, 0.8218679633240364));

    // Robot to camera 0: `new Transform3d(new Translation3d(0.23840027576590855, 0.20139212949869617, 0.34860957146726285), new Rotation3d(-0.04998098341920379, 1.0067494270931172, -0.8353690259370719))`
    // Robot to camera 1: `new Transform3d(new Translation3d(0.24873265280001045, -0.22738978917883443, 0.3606904011261275), new Rotation3d(0.02239841926409702, 0.04025359873626376, 0.8218679633240364))`
    // Robot to front left camera (centered Y compensated): `new Transform3d(new Translation3d(0.23840027576590855, 0.21439095933876529, 0.34860957146726285), new Rotation3d(-0.04998098341920379, 1.0067494270931172, -0.8353690259370719))`
    // Robot to front right camera (centered Y compensated): `new Transform3d(new Translation3d(0.24873265280001045, -0.21439095933876529, 0.3606904011261275), new Rotation3d(0.02239841926409702, 0.04025359873626376, 0.8218679633240364))`

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

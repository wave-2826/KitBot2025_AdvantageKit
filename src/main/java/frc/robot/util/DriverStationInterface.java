package frc.robot.util;

import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

import com.sun.net.httpserver.HttpServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants.ReefBranch;
import frc.robot.FieldConstants.ReefLevel;

public class DriverStationInterface {
    /**
     * The port used for the driver station interface API. The FMS firewall allows traffic for team use on ports
     * 5800-5810. 5810 is already used by NetworkTables.
     */
    private static final int DS_API_PORT = 5809;

    private static DriverStationInterface instance;

    public static DriverStationInterface getInstance() {
        if(instance == null) instance = new DriverStationInterface();
        return instance;
    }

    /**
     * The selected reef branch entry in the NetworkTables table.
     */
    private LoggedNetworkString reefBranchEntry = new LoggedNetworkString("/DriverStationInterface/ReefBranch", "A");
    /**
     * The selected reef level entry in the NetworkTables table.
     */
    private LoggedNetworkString reefLevelEntry = new LoggedNetworkString("/DriverStationInterface/ReefLevel", "L1");
    /**
     * The current robot rotation (in radians) entry in the NetworkTables table. We currently don't just use the pose
     * because deserializing it is a bit difficult.
     */
    private LoggedNetworkNumber robotRotationEntry = new LoggedNetworkNumber("/DriverStationInterface/RobotRotation",
        0);
    /**
     * The current remaining match time (in seconds) entry in the NetworkTables table.
     */
    private LoggedNetworkNumber matchTimeEntry = new LoggedNetworkNumber("/DriverStationInterface/MatchTime", -1);
    /**
     * The robot state entry in the NetworkTables table. This is used to determine if the robot is in autonomous,
     * teleop, test, or is disabled.
     */
    private LoggedNetworkString robotStateEntry = new LoggedNetworkString("/DriverStationInterface/RobotState",
        "Disabled");

    /**
     * The current robot x position (in meters) entry in the NetworkTables table. We currently don't just use the pose
     * because deserializing it is a bit difficult.
     */
    private LoggedNetworkNumber robotXEntry = new LoggedNetworkNumber("/DriverStationInterface/RobotX", 0);
    /**
     * The current robot y position (in meters) entry in the NetworkTables table. We currently don't just use the pose
     * because deserializing it is a bit difficult.
     */
    private LoggedNetworkNumber robotYEntry = new LoggedNetworkNumber("/DriverStationInterface/RobotY", 0);

    /**
     * The HTTP server used for the driver station interface API.
     */
    private HttpServer server;

    private DriverStationInterface() {
        // Start the API server
        try {
            server = HttpServer.create(new InetSocketAddress(DS_API_PORT), 0);
            server.createContext("/autoData", (exchange) -> {
                String response = getAutoData();
                // Set CORS headers so we can access this from the frontend of the dashboard
                exchange.getResponseHeaders().add("Access-Control-Allow-Origin", "*");
                exchange.getResponseHeaders().add("Access-Control-Allow-Methods", "GET, OPTIONS");

                exchange.sendResponseHeaders(200, response.length());

                try(OutputStream os = exchange.getResponseBody()) {
                    os.write(response.getBytes());
                    os.close();
                }
            });
            server.start();
        } catch(IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Gets the autonomous dashboard data. Returns a JSON string.
     * @return
     */
    private String getAutoData() {
        StringBuilder jsonData = new StringBuilder();

        try {
            jsonData.append("{\"autoChoices\": [");

            // List<String> autoNames = AutoBuilder.getAllAutoNames();
            // for(String choice : autoNames) {
            //     // Skip autos that don't make sense to show
            //     if(choice.contains("testing") || choice.toLowerCase().contains("tuning")) continue;

            //     jsonData.append("{\"name\": \"").append(choice).append("\", \"poses\": [");

            //     List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(choice);
            //     PathPlannerTrajectory simulatedPath = simulateAuto(pathGroup, DriveConstants.pathplannerConfig);

            //     double sampleInterval = 0.04;
            //     double totalTime = Math.min(simulatedPath.getTotalTimeSeconds(), 15);

            //     for(double t = 0; t < totalTime; t += sampleInterval) {
            //         PathPlannerTrajectoryState state = simulatedPath.sample(t);

            //         // @formatter:off
            //         jsonData.append("{\"x\": ").append(Math.round(state.pose.getX() * 1000) / 1000.0)
            //             .append(", \"y\": ").append(Math.round(state.pose.getY() * 1000) / 1000.0)
            //             .append(", \"rot\": ").append(Math.round(state.pose.getRotation().getRadians() * 1000) / 1000.0)
            //             .append(", \"t\": ").append(Math.round(t * 1000) / 1000.0)
            //             .append("},");
            //         // @formatter:on
            //     }

            //     if(totalTime > 0) jsonData.deleteCharAt(jsonData.length() - 1);
            //     jsonData.append("]},");
            // }

            // if(!autoNames.isEmpty()) jsonData.deleteCharAt(jsonData.length() - 1);
            jsonData.append("]}");
        } catch(Exception e) {
            System.out.println("Error loading auto data: " + e.getMessage());
        }

        return jsonData.toString();
    }

    /**
     * Gets the reef target from the dashboard.
     * @return
     */
    @AutoLogOutput(key = "/DriverStationInterface/ReefTarget")
    public ReefTarget getReefTarget() {
        try {
            return new ReefTarget(ReefBranch.valueOf(reefBranchEntry.get()), ReefLevel.valueOf(reefLevelEntry.get()));
        } catch(IllegalArgumentException e) {
            DriverStation.reportError("Invalid reef target from dashboard", false);
            return new ReefTarget(ReefBranch.A, ReefLevel.L1);
        }
    }

    /**
     * Sets the reef target on the dashboard.
     * @param target
     */
    public void setReefTarget(ReefTarget target) {
        reefBranchEntry.set(target.branch().name());
        reefLevelEntry.set(target.level().name());
    }

    /**
     * Updates the driver station dashboard with the robot position, orientation, and other state.
     */
    public void update(Pose2d pose) {
        robotRotationEntry.set(pose.getRotation().getRadians());
        robotXEntry.set(pose.getX());
        robotYEntry.set(pose.getY());
        matchTimeEntry.set(DriverStation.getMatchTime());

        if(DriverStation.isDisabled()) {
            robotStateEntry.set("Disabled");
        } else if(DriverStation.isAutonomous()) {
            robotStateEntry.set("Autonomous");
        } else if(DriverStation.isTest()) {
            robotStateEntry.set("Test");
        } else {
            robotStateEntry.set("Teleop");
        }
    }
}

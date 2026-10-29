package org.firstinspires.ftc.teamcode.autonomous;

//import com.acmerobotics.dashboard.config.Config;

@org.firstinspires.ftc.teamcode.autonomous.Config
public class AutoConstants {
    // --- DRIVING PARAMETERS ---
    public static double DRIVE_SPEED = 0.3;
    public static double TURN_SPEED = 0.4;

    // --- ROUTE A WAYPOINTS (Estimated from diagram) ---
    // Start (-63, -48) -> Launch A (-38, -30)
    public static double DIST_START_TO_LAUNCH = 5;

    // Angle to face the Hive for Launch A
    public static double ANGLE_AT_LAUNCH = 30.0;

    // Launch A (-38, -30) -> Park Pose (-57, 30)
    public static double DIST_LAUNCH_TO_PARK = 63.0;

    // Angle to turn to face the park zone after launching
    public static double ANGLE_TURN_TO_PARK = 60.0;

    // --- TIMING PARAMETERS (Milliseconds) ---
    public static long LAUNCHER_SPIN_UP_TIME = 200;
    public static long SERVO_PUSH_TIME = 500;
    public static long PAUSE_BETWEEN_COMMANDS = 250;
    public static long ALLIANCE_DELAY = 3000;
}
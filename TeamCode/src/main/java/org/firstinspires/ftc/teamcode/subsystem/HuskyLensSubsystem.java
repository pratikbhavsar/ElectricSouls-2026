package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HuskyLensSubsystem extends SubsystemBase {

    // These numbers match the order you taught the colors to the camera
    public static final int YELLOW_ID = 1;
    public static final int BLUE_ID   = 2;
    public static final int RED_ID    = 3;

    // Speeds are a fraction of the flywheel's max speed
    public static final double YELLOW_SPEED   = 0.25;
    public static final double BLUE_RED_SPEED = 0.75;

    private HuskyLens huskyLens = null;   // stays null if the camera isn't configured
    private final Telemetry telemetry;

    // 1.0 = full speed, same as the old way. Used when there is no camera.
    private double launcherSpeed = 1.0;
    private String detectedColor = "NONE";

    public HuskyLensSubsystem(HardwareMap hardwareMap, String name, Telemetry telemetry) {
        this.telemetry = telemetry;
        try {
            huskyLens = hardwareMap.get(HuskyLens.class, name);
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
            launcherSpeed = YELLOW_SPEED;   // camera found, start with the yellow speed
        } catch (Exception e) {
            huskyLens = null;
            telemetry.addData("HuskyLens", "NOT FOUND - using old fixed speed");
        }
    }

    @Override
    public void periodic() {
        // No camera? Do nothing. The flywheel just uses the old full speed.
        if (huskyLens == null) {
            return;
        }

        HuskyLens.Block[] blocks = huskyLens.blocks();

        // Pick the biggest colored block the camera sees
        HuskyLens.Block best = null;
        for (HuskyLens.Block b : blocks) {
            if (best == null || (b.width * b.height) > (best.width * best.height)) {
                best = b;
            }
        }

        if (best != null) {
            switch (best.id) {
                case YELLOW_ID:
                    launcherSpeed = YELLOW_SPEED;
                    detectedColor = "YELLOW";
                    break;
                case BLUE_ID:
                    launcherSpeed = BLUE_RED_SPEED;
                    detectedColor = "BLUE";
                    break;
                case RED_ID:
                    launcherSpeed = BLUE_RED_SPEED;
                    detectedColor = "RED";
                    break;
                default:
                    detectedColor = "UNKNOWN (id " + best.id + ")";
                    break;
            }
        } else {
            detectedColor = "NONE";
        }

        telemetry.addData("HuskyLens Color", detectedColor);
        telemetry.addData("Launcher Speed", launcherSpeed);
    }

    public double getLauncherSpeed() {
        return launcherSpeed;
    }
}
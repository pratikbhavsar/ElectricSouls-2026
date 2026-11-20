package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Autonomous program to drive forward 12 inches and stop.
 */
@Autonomous(name = "Drive Forward 12 Inches", group = "Autonomous")
public class MoveForwardAuto extends AllianceAwareAuto {

    public MoveForwardAuto() {
        super(true); // Default Red alliance context
    }

    @Override
    public void runOpMode() {
        initHardware();
        telemetry.addData("Status", "Initialized. Waiting for start.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Drive forward 12 inches (+12.0) at 0.25 slow power with heading hold
            driveStraight(0.25, 12.0);
        }
    }
}

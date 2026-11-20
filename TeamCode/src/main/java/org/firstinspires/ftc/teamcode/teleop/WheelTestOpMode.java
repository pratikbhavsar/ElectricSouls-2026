package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Diagnostic OpMode to test each mecanum wheel individually.
 * - Hold A: Front Left
 * - Hold B: Front Right
 * - Hold X: Back Left
 * - Hold Y: Back Right
 */
@TeleOp(name = "Wheel Test OpMode", group = "Diagnostics")
public class WheelTestOpMode extends LinearOpMode {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        // Use proven motor directions matching drivetrain config
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addData("Status", "Wheel Test Initialized.");
        telemetry.addData("Controls", "Hold A=FL, B=FR, X=BL, Y=BR (30% Power)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double pFl = 0.0;
            double pFr = 0.0;
            double pBl = 0.0;
            double pBr = 0.0;

            String activeWheel = "None (Release)";

            if (gamepad1.a) {
                pFl = 0.3;
                activeWheel = "Front Left (A)";
            } else if (gamepad1.b) {
                pFr = 0.3;
                activeWheel = "Front Right (B)";
            } else if (gamepad1.x) {
                pBl = 0.3;
                activeWheel = "Back Left (X)";
            } else if (gamepad1.y) {
                pBr = 0.3;
                activeWheel = "Back Right (Y)";
            }

            frontLeft.setPower(pFl);
            frontRight.setPower(pFr);
            backLeft.setPower(pBl);
            backRight.setPower(pBr);

            telemetry.addData("Active Wheel", activeWheel);
            telemetry.addData("FL Ticks / Vel", "%d / %.1f", frontLeft.getCurrentPosition(), frontLeft.getVelocity());
            telemetry.addData("FR Ticks / Vel", "%d / %.1f", frontRight.getCurrentPosition(), frontRight.getVelocity());
            telemetry.addData("BL Ticks / Vel", "%d / %.1f", backLeft.getCurrentPosition(), backLeft.getVelocity());
            telemetry.addData("BR Ticks / Vel", "%d / %.1f", backRight.getCurrentPosition(), backRight.getVelocity());
            telemetry.update();
        }
    }
}

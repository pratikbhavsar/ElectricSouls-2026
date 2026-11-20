package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Mecanum drive subsystem for TeleOp, updated to match the exact motor configuration
 * and kinematics of AllianceAwareAuto (all 4 motors FORWARD).
 */
public class MecanumDriveSubsystem extends SubsystemBase {
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;

    private final Telemetry telemetry;
    private final IMU imu;

    public MecanumDriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        frontLeft = getDcMotorSafely(hardwareMap, "frontLeft", "front_left_drive", "left_front_drive", "leftFront");
        frontRight = getDcMotorSafely(hardwareMap, "frontRight", "front_right_drive", "right_front_drive", "rightFront");
        backLeft = getDcMotorSafely(hardwareMap, "backLeft", "back_left_drive", "left_back_drive", "leftBack");
        backRight = getDcMotorSafely(hardwareMap, "backRight", "back_right_drive", "right_back_drive", "rightBack");

        // All 4 motors set to FORWARD matching AllianceAwareAuto
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
    }

    private DcMotorEx getDcMotorSafely(HardwareMap hwMap, String... names) {
        for (String name : names) {
            try {
                return hwMap.get(DcMotorEx.class, name);
            } catch (Exception ignored) {}
        }
        throw new RuntimeException("Could not find motor with any of names: " + java.util.Arrays.toString(names));
    }

    public void driveFieldRelative(double forward, double right, double rotate) {
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive(newForward, newRight, rotate);
    }

    public void drive(double forward, double right, double rotate) {
        double rot = rotate * 0.6;
        double f = forward;
        double s = right;

        // Kinematics matching AllianceAwareAuto (all 4 motors FORWARD)
        double pFl = f + s + rot;
        double pFr = -(f - s - rot);
        double pBl = f - s + rot; // Updated for FWD backLeft
        double pBr = f + s - rot;

        double maxM = Math.max(1.0, Math.max(Math.max(Math.abs(pFl), Math.abs(pFr)),
                Math.max(Math.abs(pBl), Math.abs(pBr))));

        double maxSpeed = 0.8;

        frontLeft.setPower(maxSpeed * (pFl / maxM));
        frontRight.setPower(maxSpeed * (pFr / maxM));
        backLeft.setPower(maxSpeed * (pBl / maxM));
        backRight.setPower(maxSpeed * (pBr / maxM));
    }
}

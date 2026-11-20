package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.PushBallSubsystem;

/**
 * Base abstract class and concrete Autonomous OpModes for Red & Blue alliances
 * (Near vs Far starting positions), utilizing the 24" tile map grid,
 * always driving in reverse (launcher in back, intake in front), and alliance-aware sign flipping.
 */
public abstract class AllianceAwareAuto extends LinearOpMode {

    protected DcMotorEx fl, fr, bl, br;
    protected IMU imu;

    protected static final double TICKS_PER_REV = 537.7;
    protected static final double WHEEL_DIAMETER_IN = 3.77953;
    protected static final double TICKS_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER_IN * Math.PI);
    protected static final double STRAFE_MULTIPLIER = 1.15;

    protected final boolean isRed;
    protected final double allianceSign; // +1.0 for Red, -1.0 for Blue

    protected AllianceAwareAuto(boolean isRed) {
        this.isRed = isRed;
        this.allianceSign = isRed ? 1.0 : -1.0;
    }

    protected void initHardware() {
        fl = getDcMotorSafely(hardwareMap, "frontLeft", "front_left_drive", "left_front_drive", "leftFront");
        fr = getDcMotorSafely(hardwareMap, "frontRight", "front_right_drive", "right_front_drive", "rightFront");
        bl = getDcMotorSafely(hardwareMap, "backLeft", "back_left_drive", "left_back_drive", "leftBack");
        br = getDcMotorSafely(hardwareMap, "backRight", "back_right_drive", "right_back_drive", "rightBack");

        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD); // Updated to FORWARD as requested
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();
    }

    private DcMotorEx getDcMotorSafely(HardwareMap hwMap, String... names) {
        for (String name : names) {
            try {
                return hwMap.get(DcMotorEx.class, name);
            } catch (Exception ignored) {}
        }
        throw new RuntimeException("Could not find motor with any of names: " + java.util.Arrays.toString(names));
    }

    /** Drive straight (+ forward, - reverse) with heading hold disabled temporarily for debugging */
    protected void driveStraight(double power, double inches) {
        resetEncoders();
        int targetTicks = (int) Math.round(Math.abs(inches) * TICKS_PER_INCH);
        ElapsedTime timer = new ElapsedTime();
        double maxTimeSec = (Math.abs(inches) * 0.08) / Math.max(0.05, power);
        double direction = Math.signum(inches);

        while (opModeIsActive() && timer.seconds() < maxTimeSec) {
            int avgTicks = getAverageEncoderTicks();
            if (avgTicks >= targetTicks) break;

            double hold = 0.0; // Disabled heading hold to isolate pure motor powers

            setMecanumPowers(power * direction, 0.0, hold);
            telemetry.addData("Alliance", isRed ? "RED (+1)" : "BLUE (-1)");
            telemetry.addData("Drive Ticks", "%d / %d", avgTicks, targetTicks);
            telemetry.update();
        }
        stopMotors();
    }

    /**
     * Strafe (+ right, - left).
     * @param allianceAware if true, automatically multiplies inches by allianceSign
     */
    protected void strafe(double power, double inches, boolean allianceAware) {
        resetEncoders();
        double effectiveInches = allianceAware ? inches * allianceSign : inches;

        int targetTicks = (int) Math.round(Math.abs(effectiveInches) * STRAFE_MULTIPLIER * TICKS_PER_INCH);
        ElapsedTime timer = new ElapsedTime();
        double maxTimeSec = (Math.abs(effectiveInches) * STRAFE_MULTIPLIER * 0.08) / Math.max(0.05, power);
        double direction = Math.signum(effectiveInches);

        while (opModeIsActive() && timer.seconds() < maxTimeSec) {
            int avgTicks = getAverageEncoderTicks();
            if (avgTicks >= targetTicks) break;

            double hold = 0.0;

            setMecanumPowers(0.0, power * direction, hold);
            telemetry.addData("Alliance", isRed ? "RED (+1)" : "BLUE (-1)");
            telemetry.addData("Strafe Ticks", "%d / %d", avgTicks, targetTicks);
            telemetry.update();
        }
        stopMotors();
    }

    /** Start intake and launch pollen balls using PushBallSubsystem & IntakeSubsystem before movement */
    protected void launchPollens() {
        PushBallSubsystem launcherSubsystem = new PushBallSubsystem(hardwareMap, "launcher", telemetry);
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);

        telemetry.addData("Status", "Starting intake & spinning up launcher...");
        telemetry.update();

        intakeSubsystem.in();       // Start intake motor and servos
        launcherSubsystem.launch(); // Spin up launcher flywheel
        sleep(1500);                 // Spin-up time

        int shots = 4;
        for (int i = 0; i < shots; i++) {
            ElapsedTime waitTimer = new ElapsedTime();
            waitTimer.reset();
            while (opModeIsActive() && launcherSubsystem.getVelocity() < 700.0 && waitTimer.seconds() < 2.0) {
                telemetry.addData("Velocity", "%.1f (Waiting for >= 700)", launcherSubsystem.getVelocity());
                telemetry.update();
                sleep(20);
            }

            telemetry.addData("Status", "Firing pollen ball %d of %d (Vel: %.1f)", i + 1, shots, launcherSubsystem.getVelocity());
            telemetry.update();

            launcherSubsystem.pushBall();
            sleep(500);  // Push servo run time
            launcherSubsystem.stopServo();
            sleep(700);  // Flywheel recovery time between shots
        }

        launcherSubsystem.stop();
        intakeSubsystem.stop(); // Stop intake when done launching
        telemetry.addData("Status", "Launch sequence complete.");
        telemetry.update();
    }

    protected void setMecanumPowers(double f, double s, double hold) {
        double pFl = f + s + hold;
        double pFr = -(f - s - hold);
        double pBl = f - s + hold; // Updated for FWD backLeft
        double pBr = f + s - hold;

        double maxM = Math.max(1.0, Math.max(Math.max(Math.abs(pFl), Math.abs(pFr)),
                Math.max(Math.abs(pBl), Math.abs(pBr))));

        double finalFl = pFl / maxM;
        double finalFr = pFr / maxM;
        double finalBl = pBl / maxM;
        double finalBr = pBr / maxM;

        fl.setPower(finalFl);
        fr.setPower(finalFr);
        bl.setPower(finalBl);
        br.setPower(finalBr);

        telemetry.addData("Power FL | FR", "%.2f | %.2f", finalFl, finalFr);
        telemetry.addData("Power BL | BR", "%.2f | %.2f", finalBl, finalBr);
    }

    protected int getAverageEncoderTicks() {
        return (Math.abs(fl.getCurrentPosition()) +
                Math.abs(fr.getCurrentPosition()) +
                Math.abs(bl.getCurrentPosition()) +
                Math.abs(br.getCurrentPosition())) / 4;
    }

    protected void resetEncoders() {
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    protected void stopMotors() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    // ==================== 4 CONCRETE OPMODES (RED / BLUE x NEAR / FAR) ====================

    @Autonomous(name = "Auto - RED Near Parking", group = "Autonomous")
    public static class RedNear extends AllianceAwareAuto {
        public RedNear() { super(true); }

        @Override
        public void runOpMode() {
            initHardware();
            waitForStart();

            if (opModeIsActive()) {
                // 1. Launch pollens & start intake BEFORE any movements
                launchPollens();
                sleep(250);

                // 2. 1 tile = 24 inches. Always drive in reverse (-24 in)
                driveStraight(0.3, -24.0);
                sleep(250);

                // 3. Strafe (allianceAware = true automatically flips direction)
                strafe(0.3, -40.0, true);
            }
        }
    }

    @Autonomous(name = "Auto - RED Far From Parking", group = "Autonomous")
    public static class RedFar extends AllianceAwareAuto {
        public RedFar() { super(true); }

        @Override
        public void runOpMode() {
            initHardware();
            waitForStart();

            if (opModeIsActive()) {
                // 1. Launch pollens & start intake BEFORE any movements
                launchPollens();
                sleep(250);

                // 2. 1 tile reverse
                driveStraight(0.3, -24.0);
                sleep(250);

                // 3. Cross 2 tiles across the field (48 inches)
                strafe(0.4, 40.0, true);
                sleep(250);

                // 4. Park in reverse (2.5 tiles = 60 inches)
                driveStraight(0.3, -60.0);
            }
        }
    }

    @Autonomous(name = "Auto - BLUE Near Parking", group = "Autonomous")
    public static class BlueNear extends AllianceAwareAuto {
        public BlueNear() { super(false); }

        @Override
        public void runOpMode() {
            initHardware();
            waitForStart();

            if (opModeIsActive()) {
                // 1. Launch pollens & start intake BEFORE any movements
                launchPollens();
                sleep(250);

                // 2. 1 tile reverse (-24 inches)
                driveStraight(0.3, -24.0);
                sleep(250);

                // 3. Strafe (allianceAware = true automatically flips Red's right to Blue's left)
                strafe(0.3, 40.0, true);
            }
        }
    }

    @Autonomous(name = "Auto - BLUE Far From Parking", group = "Autonomous")
    public static class BlueFar extends AllianceAwareAuto {
        public BlueFar() { super(false); }

        @Override
        public void runOpMode() {
            initHardware();
            waitForStart();

            if (opModeIsActive()) {
                // 1. Launch pollens & start intake BEFORE any movements
                launchPollens();
                sleep(250);

                // 2. 1 tile reverse
                driveStraight(0.3, -24.0);
                sleep(250);

                // 3. Cross 2 tiles across field (auto-flipped for Blue)
                strafe(0.4, -40.0, true);
                sleep(250);

                // 4. Park in reverse (2.5 tiles = 60 inches)
                driveStraight(0.3, -60.0);
            }
        }
    }
}

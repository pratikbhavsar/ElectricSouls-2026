package org.firstinspires.ftc.teamcode.autonomous;

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
 * 4-wheel mecanum autonomous drive: encoder distance moves (forward + strafe) with IMU heading hold.
 * Self-contained: needs only the four motors and the IMU from your robot configuration.
 */
public class AutoDriveSubsystem extends SubsystemBase {
    private final DcMotorEx fl, fr, bl, br;
    private final IMU imu;
    private final Telemetry telemetry;

    // goBILDA 312 RPM (537.7 ticks/rev) with 96 mm mecanum wheels. Verify yours.
    private static final double TICKS_PER_REV = 100;
    private static final double WHEEL_DIAMETER_IN = 3.77953;
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER_IN * Math.PI);

    // TUNE: command a 24 in strafe, measure real travel, multiplier = commanded / actual
    private static final double STRAFE_MULTIPLIER = 1.15;

    // TUNE
    private static final double KP_DRIVE = 0.004;     // power per tick of error
    private static final double KP_HEADING = 0.02;    // power per degree
    private static final double MIN_POWER = 0.08;     // overcome friction
    private static final int TICK_TOLERANCE = 15;     // about 0.3 in
    private static final boolean USE_HEADING_HOLD = true;

    private int flT, frT, blT, brT;

    public AutoDriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        br = hardwareMap.get(DcMotorEx.class, "backRight");

        // Left forward, right reversed (reverses the encoders too, so no runaway)
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Same hub orientation as your teleop MecanumDriveSubsystem
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw(); // heading 0 = the way the robot faces at init
    }
//hellothere
    private double yaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private double headingError() {
        double e = 0 - yaw();
        while (e > 180) e -= 360;
        while (e < -180) e += 360;
        return e;
    }

    private double pctrl(double error, double max) {
        if (Math.abs(error) <= TICK_TOLERANCE) return 0;
        double p = Math.max(-max, Math.min(max, KP_DRIVE * error));
        if (Math.abs(p) < MIN_POWER) p = Math.signum(error) * MIN_POWER;
        return p;
    }

    private void setPowers(double a, double b, double c, double d) {
        double m = Math.max(1.0, Math.max(Math.max(Math.abs(a), Math.abs(b)),
                Math.max(Math.abs(c), Math.abs(d))));
        fl.setPower(a / m);
        fr.setPower(b / m);
        bl.setPower(c / m);
        br.setPower(d / m);
    }

    /** forwardIn: + forward. strafeIn: + right. Both relative to where the robot is now. */
    public void startMove(double forwardIn, double strafeIn) {
        double f = forwardIn * TICKS_PER_INCH;
        double s = strafeIn * TICKS_PER_INCH * STRAFE_MULTIPLIER;
        flT = fl.getCurrentPosition() + (int) Math.round(f + s);
        frT = fr.getCurrentPosition() + (int) Math.round(f - s);
        blT = bl.getCurrentPosition() + (int) Math.round(f - s);
        brT = br.getCurrentPosition() + (int) Math.round(f + s);
    }

    /** Call every loop while moving. */
    public void updateMove(double maxPower) {
        double hold = USE_HEADING_HOLD ? -KP_HEADING * headingError() : 0; // + = clockwise
        double pFl = pctrl(flT - fl.getCurrentPosition(), maxPower);
        double pFr = pctrl(frT - fr.getCurrentPosition(), maxPower);
        double pBl = pctrl(blT - bl.getCurrentPosition(), maxPower);
        double pBr = pctrl(brT - br.getCurrentPosition(), maxPower);
        setPowers(pFl + hold, pFr - hold, pBl + hold, pBr - hold);
    }

    public boolean moveAtTarget() {
        return Math.abs(flT - fl.getCurrentPosition()) <= TICK_TOLERANCE
                && Math.abs(frT - fr.getCurrentPosition()) <= TICK_TOLERANCE
                && Math.abs(blT - bl.getCurrentPosition()) <= TICK_TOLERANCE
                && Math.abs(brT - br.getCurrentPosition()) <= TICK_TOLERANCE;
    }

    public void stop() {
        setPowers(0, 0, 0, 0);
    }

    @Override
    public void periodic() {
        telemetry.addData("Yaw", "%.1f", yaw());
        telemetry.addData("Ticks FL/FR/BL/BR", "%d %d %d %d",
                fl.getCurrentPosition(), fr.getCurrentPosition(),
                bl.getCurrentPosition(), br.getCurrentPosition());
    }
}
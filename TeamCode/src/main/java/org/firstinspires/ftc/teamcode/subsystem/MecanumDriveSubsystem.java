
package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Robot: Field Relative Mecanum Drive", group = "Robot")
@Disabled
public class MecanumDriveSubsystem extends SubsystemBase {
    // This declares the four motors needed
    private MotorEx frontLeft;
    private MotorEx frontRight;
    private MotorEx backLeft;
    private MotorEx backRight;

    private Telemetry telemetry;

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    public MecanumDriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Match the hardware map names from your previous file ("backLeft", "backRight")
        this.backLeft = new MotorEx(hardwareMap, "backLeft");
        this.backRight = new MotorEx(hardwareMap, "backRight");
        this.frontLeft = new MotorEx(hardwareMap, "frontLeft");
        this.frontRight = new MotorEx(hardwareMap, "frontRight");

        backLeft.setInverted(true);
        backRight.setInverted(true);
        frontLeft.setInverted(true);
        frontRight.setInverted(true);

        // Switch to RawPower to accept values from -1.0 to 1.0 (percent power)
        backLeft.setRunMode(Motor.RunMode.RawPower);
        backRight.setRunMode(Motor.RunMode.RawPower);
        frontRight.setRunMode(Motor.RunMode.RawPower);
        frontLeft.setRunMode(Motor.RunMode.RawPower);

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.

        //backLeft.setDirection(MotorEx.Direction.REVERSE);
        //frontLeft.setDirection(MotorEx.Direction.REVERSE);

        //frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));


    }

    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    public void drive(double forward, double right, double rotate) {

        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeft.set(maxSpeed * (frontLeftPower / maxPower));
        frontRight.set(maxSpeed * (frontRightPower / maxPower));
        backLeft.set(maxSpeed * (backLeftPower / maxPower));
        backRight.set(maxSpeed * (backRightPower / maxPower));

    }
}



        // This needs to be changed to match the orientation on your robot

package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveSubsystem extends SubsystemBase {
    private MotorEx backLeft;
    private MotorEx backRight;
    //private MotorEx frontLeft;
    //private MotorEx frontRight;
    private Telemetry telemetry;

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Match the hardware map names from your previous file ("backLeft", "backRight")
        this.backLeft = new MotorEx(hardwareMap, "backLeft");
        this.backRight = new MotorEx(hardwareMap, "backRight");
        //this.frontLeft = new MotorEx(hardwareMap, "frontLeft");
        //this.frontRight = new MotorEx(hardwareMap, "frontRight");

        backLeft.setInverted(false);
        backRight.setInverted(true);
        //frontRight.setInverted(true);
        //frontLeft.setInverted(true);0
        // Switch to RawPower to accept values from -1.0 to 1.0 (percent power)
        backLeft.setRunMode(Motor.RunMode.RawPower);
        backRight.setRunMode(Motor.RunMode.RawPower);
        //frontLeft.setRunMode(Motor.RunMode.RawPower);
        //frontRight.setRunMode(Motor.RunMode.RawPower);


    }

    /**
     * Drives the robot using POV/Arcade control logic.
     * @param rotate Forward/backward movement (-1.0 to 1.0)
     * @param forward Turning movement (-1.0 to 1.0)
     */
    public void drive(double forward, double rotate) {
        double left  = forward + rotate;
        double right = forward - rotate;

        // Normalize the values so neither exceed +/- 1.0
        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        // Send calculated power levels to the motors
        backLeft.set(left);
        backRight.set(right);
        //frontLeft.set(left);
        //frontRight.set(right);

        // Telemetry updates
        telemetry.addData("Drive Motors", "Left (%.2f), Right (%.2f)", left, right);
        telemetry.update();
    }

    public void stop() {
        backLeft.stopMotor();
        backRight.stopMotor();
        //frontRight.stopMotor();
        //frontLeft.stopMotor();
    }
}
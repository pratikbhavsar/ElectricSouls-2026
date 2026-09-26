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
    private Telemetry telemetry;

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Match the hardware map names from your previous file ("backLeft", "backRight")
        this.backLeft = new MotorEx(hardwareMap, "backLeft");
        this.backRight = new MotorEx(hardwareMap, "backRight");

        // Reversing left side matching the logic in the sample OpMode
        backLeft.setInverted(true);
        backRight.setInverted(false);

        // Switch to RawPower to accept values from -1.0 to 1.0 (percent power)
        backLeft.setRunMode(Motor.RunMode.RawPower);
        backRight.setRunMode(Motor.RunMode.RawPower);
    }

    /**
     * Drives the robot using POV/Arcade control logic.
     * @param drive Forward/backward movement (-1.0 to 1.0)
     * @param turn Turning movement (-1.0 to 1.0)
     */
    public void drive(double forward, double rotate) {
/* Set the drive and turn variables to follow the joysticks on the gamepad.
            the joysticks decrease as you push them up. So reverse the Y axis. */



            /* Here we "mix" the input channels together to find the power to apply to each motor.
            The both motors need to be set to a mix of how much you're retesting the robot move
            forward, and how much you're requesting the robot turn. When you ask the robot to rotate
            the right and left motors need to move in opposite directions. So we will add rotate to
            forward for the left motor, and subtract rotate from forward for the right motor. */

        double left  = forward + rotate;
        double right = forward - rotate;

        /* Normalize the values so neither exceed +/- 1.0 */
        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        // Send calculated power levels to the motors
        backLeft.set(left);
        backRight.set(right);

        // Optional telemetry updates
        telemetry.addData("Drive Motors", "Left (%.2f), Right (%.2f)", left, right);
        telemetry.update();
    }

    public void stop() {
        backLeft.stopMotor();
        backRight.stopMotor();
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorBoard {
    private DcMotor motor;
    private double ticksPerRotation;

    public void init(HardwareMap hwMap) {
        // Name must match the configuration file EXACTLY
        motor = hwMap.get(DcMotor.class, "motor");

        // Reset the encoder to zero, then run with encoder feedback
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Ticks per revolution for the motor type chosen in the config
        ticksPerRotation = motor.getMotorType().getTicksPerRev();
    }

    // speed: -1.0 (full reverse) to 1.0 (full forward)
    public void setMotorSpeed(double speed) {
        motor.setPower(speed);
    }

    public double getMotorRotations() {
        return motor.getCurrentPosition() / ticksPerRotation;
    }

    public int getMotorTicks() {
        return motor.getCurrentPosition();
    }

    public void setBrake(boolean brake) {
        motor.setZeroPowerBehavior(brake
                ? DcMotor.ZeroPowerBehavior.BRAKE
                : DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

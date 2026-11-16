package org.firstinspires.ftc.teamcode.autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class MotorCode {
    private DcMotor motor;
    private MotorEx motorEx;
    private double ticksPerRotation;

    public void init(HardwareMap HWMap) {
        motor = HWMap.get(DcMotor.class,"motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ticksPerRotation = motor.getMotorType().getTicksPerRev();

    }
    public void setMotorSpeed(double speed) {
        motor.setPower(speed);

    }
    public int getMotorTicks() {
        return motor.getCurrentPosition();

    }
    public double getMotorRotations() {
        return motor.getCurrentPosition() / ticksPerRotation;

    }
}

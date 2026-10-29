package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ServoWheelSubsystem {
    private CRServo intakeServoLeft;
    private CRServo intakeServoRight;
    private static final double SERVO_POWER = -0.5; //tune this

    public ServoWheelSubsystem(final HardwareMap hardwareMap, final String motorName, Telemetry telemetry) {
        this.intakeServoLeft = hardwareMap.get(CRServo.class, "servoLeft");
        this.intakeServoRight = hardwareMap.get(CRServo.class, "servoRight");
        intakeServoRight.setDirection(CRServoImpl.Direction.REVERSE);
        intakeServoLeft.setDirection(CRServoImpl.Direction.FORWARD);

    }

    public ServoWheelSubsystem(HardwareMap hardwareMap,  Telemetry telemetry) {
    }

    public void setIntakeServos() {
        intakeServoLeft.setPower(SERVO_POWER);
        intakeServoLeft.setPower(SERVO_POWER);
    }

    public void stopServos() {
        intakeServoRight.setPower(0);
        intakeServoLeft.setPower(0);

    }
}

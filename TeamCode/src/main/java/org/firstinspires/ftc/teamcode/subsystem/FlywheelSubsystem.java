package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.LED;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase {

    private final DcMotorEx motor;
    private final Telemetry telemetry;
    private final LED greenLed;
    private final LED redLed;

    //private static final double LAUNCH_POWER = 0.09;// tune this
    private static final int MAX_VELOCITY = 1200;
    public FlywheelSubsystem(HardwareMap hardwareMap, String motorName, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.greenLed = (LED) hardwareMap.get(LED.class, "greenLed");
        this.redLed = (LED) hardwareMap.get(LED.class, "redLed");
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        // motor.setDirection(DcMotorSimple.Direction.REVERSE); // uncomment if it spins backwards
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setVelocityPIDFCoefficients(300, 0, 0, 0); // only change 1st value
        telemetry.addData("MAX VELOCITY:", MAX_VELOCITY);
        telemetry.update();
        redLed.off();
        greenLed.off();

    }

    public void launch() {
        motor.setVelocity(MAX_VELOCITY);
    }

    public void stop() {
        motor.setPower(0);
        greenLed.off();
    }

    @Override
    public void periodic() {
//        telemetry.addData("Flywheel power", motor.getPower());
        telemetry.addData("Current Velocity", motor.getVelocity());
        telemetry.update();
        setgreenLed();
    }

    public void setgreenLed() {
        if(motor.getVelocity()>=1050) {
            greenLed.on();
        } else {
            greenLed.off();
        }
    }
}

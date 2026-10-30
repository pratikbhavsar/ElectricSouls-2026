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

    private static final double LAUNCH_POWER = 1.0;// tune this
    private static final int MAX_VELOCITY = 1200;
    public FlywheelSubsystem(HardwareMap hardwareMap, String motorName, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.greenLed = (LED) hardwareMap.get("greenLed");
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        // motor.setDirection(DcMotorSimple.Direction.REVERSE); // uncomment if it spins backwards
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void launch() {
        motor.setVelocity(MAX_VELOCITY);
    }

    public void stop() {
        motor.setPower(0);
    }

    @Override
    public void periodic() {
        telemetry.addData("Flywheel power", motor.getPower());
        setgreenLed();
    }

    public void setgreenLed() {
        if(motor.getVelocity()>MAX_VELOCITY) {
            greenLed.on();
        } else {
            greenLed.off();
        }
    }
}

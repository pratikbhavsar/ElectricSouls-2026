package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase {

    private final DcMotorEx motor;
    private final Telemetry telemetry;

    private static final double LAUNCH_POWER = 1.0; // tune this

    public FlywheelSubsystem(HardwareMap hardwareMap, String motorName, Telemetry telemetry) {
        this.telemetry = telemetry;
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        // motor.setDirection(DcMotorSimple.Direction.REVERSE); // uncomment if it spins backwards
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    public void launch() {
        motor.setPower(LAUNCH_POWER);
    }

    public void stop() {
        motor.setPower(0);
    }

    @Override
    public void periodic() {
        telemetry.addData("Flywheel power", motor.getPower());
    }
}

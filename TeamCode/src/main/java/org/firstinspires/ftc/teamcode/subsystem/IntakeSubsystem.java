package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class IntakeSubsystem extends SubsystemBase {
    private final DcMotorEx intakemotor;
    private final CRServo intakeservoleft;
    private final CRServo intakeservoright;

    private final Telemetry telemetry;

    private final VoltageSensor batteryVoltageSensor;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        intakemotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        this.telemetry = telemetry;

        intakemotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.intakeservoleft = hardwareMap.get(CRServo.class,"servoLeft");
        this.intakeservoright = hardwareMap.get(CRServo.class,"servoRight");

        this.intakeservoright.setDirection(DcMotorSimple.Direction.REVERSE);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

    }

    public void in ()
    {
        intakemotor.setVelocity(500);
        intakeservoleft.setPower(1);
        intakeservoright.setPower(1);
    }
    public void stop (){
        intakemotor.setVelocity(0);
        intakeservoleft.setPower(0);
        intakeservoright.setPower(0);
    }
    @Override
    public void periodic() {
        double currentAmps = intakemotor.getCurrent(CurrentUnit.AMPS);
        double voltageVolts = batteryVoltageSensor.getVoltage();
        double powerWatts = currentAmps * voltageVolts;
//        this.telemetry.addData("Intake Current (A)", currentAmps);
//        this.telemetry.addData("Intake Power (W)", powerWatts);
//        this.telemetry.update();
    }
}
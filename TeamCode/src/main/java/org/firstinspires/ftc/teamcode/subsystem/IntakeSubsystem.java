package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class IntakeSubsystem extends SubsystemBase {
    private final MotorEx intakemotor;
//    private final CRServo intakeservoleft;
//    private final CRServo intakeservoright;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        this.intakemotor = new MotorEx(hardwareMap, "intakeMotor");
        intakemotor.setInverted(true);
//        this.intakeservoleft = hardwareMap.get(CRServo.class,"servoLeft");
//        this.intakeservoright = hardwareMap.get(CRServo.class,"servoRight");

//        this.intakeservoright.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void in ()
    {
        intakemotor.setVelocity(1500);
//        intakeservoleft.setPower(0.5);
//        intakeservoright.setPower(0.5);
    }
    public void stop (){
        intakemotor.setVelocity(0);
//        intakeservoleft.setPower(0);
//        intakeservoright.setPower(0);
    }
}

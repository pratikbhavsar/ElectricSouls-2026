package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PushBallSubsystem extends SubsystemBase {
    private final DcMotorEx launcherMotor;

    private final CRServo pushBall;
    private final Telemetry telemetry;

    // Target velocities in Encoder Ticks per Second
    private static final double TARGET_LAUNCH_VELOCITY = 1300.0;

    public PushBallSubsystem(final HardwareMap hardwareMap, final String motorName, Telemetry telemetry) {
        // Initialize the single MotorEx instance
        this.launcherMotor = hardwareMap.get(DcMotorEx.class, motorName);
        this.telemetry = telemetry;
        this.pushBall =  hardwareMap.get(CRServo.class, "pushBall");
        // Switch RunMode to Velocity control for closed-loop PID precision
//        this.launcherMotor.setRunMode(Motor.RunMode.VelocityControl);
        this.launcherMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        this.launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.launcherMotor.setVelocityPIDFCoefficients(300, 0, 0, 0); //
        // Zero power behavior configuration
//        this.launcherMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

    }


    public void pushBall(){
        // Adjust rotation direction if needed based on mounting layout (true or false)
        this.launcherMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("Setting Servo power To 1","");
        telemetry.update();
        pushBall.setPower(1.0);
        telemetry.addData("launcher servo","power %.2f", pushBall.getPower());
        telemetry.update();
    }


    public void reverse() {
        this.launcherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Setting Servo power To 1","");
        telemetry.update();
        pushBall.setPower(1.0);
        telemetry.addData("launcher servo","power %.2f", pushBall.getPower());
        telemetry.update();
    }

    public void stopServo () {
        pushBall.setPower(0);
        telemetry.addData("launcher servo","power %.2f", pushBall.getPower());
        telemetry.update();
    }
    private void setVelocity(double ticksPerSecond) {
        launcherMotor.setVelocity(ticksPerSecond);
    }

    /**
     * Spins up the flywheel to the target launching velocity.
     */
    public void launch() {
        launcherMotor.setVelocity(TARGET_LAUNCH_VELOCITY);
    }

    public double getVelocity() {
        return launcherMotor.getVelocity();
    }

    /**
     * Stops the motor immediately.
     */
    public void stop() {
        launcherMotor.setVelocity(0);
    }

}

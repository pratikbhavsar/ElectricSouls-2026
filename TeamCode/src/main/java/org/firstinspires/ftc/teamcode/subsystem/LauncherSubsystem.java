package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LauncherSubsystem extends SubsystemBase {
    private final MotorEx launcherMotor;

    private final CRServo launcherServo;
    private final Telemetry telemetry;

    // Target velocities in Encoder Ticks per Second
    private static final double TARGET_LAUNCH_VELOCITY = 1000.0;

    public LauncherSubsystem(final HardwareMap hardwareMap, final String motorName, Telemetry telemetry) {
        // Initialize the single MotorEx instance
        this.launcherMotor = new MotorEx(hardwareMap, motorName);
        this.telemetry = telemetry;
        this.launcherServo =  hardwareMap.get(CRServo.class, "launcherServo");
        // Switch RunMode to Velocity control for closed-loop PID precision
        this.launcherMotor.setRunMode(Motor.RunMode.VelocityControl);

        // Zero power behavior configuration
        this.launcherMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        // Adjust rotation direction if needed based on mounting layout (true or false)
        this.launcherMotor.setInverted(false);
    }


    public void pushBall(){
        telemetry.addData("Setting Servo power To 1","");
        telemetry.update();
        launcherServo.setPower(1.0);
        telemetry.addData("launcher servo","power %.2f", launcherServo.getPower());
        telemetry.update();
    }
    public void stopServo () {
        launcherServo.setPower(0);
        telemetry.addData("launcher servo","power %.2f", launcherServo.getPower());
        telemetry.update();
    }
    private void setVelocity(double ticksPerSecond) {
        launcherMotor.setVelocity(ticksPerSecond);
    }

    /**
     * Spins up the flywheel to the target launching velocity.
     */
    public void launch() {
        setVelocity(TARGET_LAUNCH_VELOCITY);
    }

    /**
     * Stops the motor immediately.
     */
    public void stop() {
        launcherMotor.stopMotor();
    }

}

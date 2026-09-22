package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class LauncherSubsystem extends SubsystemBase {
    private final MotorEx launcherMotor;

    // Target velocities in Encoder Ticks per Second
    private static final double TARGET_LAUNCH_VELOCITY = 500.0;

    public LauncherSubsystem(final HardwareMap hardwareMap, final String motorName) {
        // Initialize the single MotorEx instance
        this.launcherMotor = new MotorEx(hardwareMap, motorName);

        // Switch RunMode to Velocity control for closed-loop PID precision
        this.launcherMotor.setRunMode(Motor.RunMode.VelocityControl);

        // Zero power behavior configuration
        this.launcherMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        // Adjust rotation direction if needed based on mounting layout (true or false)
        this.launcherMotor.setInverted(false);
    }

    /**
     * Sets a precise target velocity for the single flywheel motor.
     * @param ticksPerSecond Target speed in encoder counts per second
     */
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

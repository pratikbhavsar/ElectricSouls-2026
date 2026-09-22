package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.StartEndCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystem.LauncherSubsystem;

@TeleOp(name = "Launcher Subsystem")
public class ConceptLauncher extends CommandOpMode {

    // 1. Declare your subsystems and controllers here
    private LauncherSubsystem launcher;
    private GamepadEx gamepadOne;

    @Override
    public void initialize() {
        // 2. ALWAYS reset the scheduler when the OpMode initializes
        CommandScheduler.getInstance().reset();

        // 3. Instantiate hardware and setup your button bindings
        launcher = new LauncherSubsystem(hardwareMap, "launcher");
        gamepadOne = new GamepadEx(gamepad1);

        // Schedule / Register your subsystems here...
        gamepadOne.getGamepadButton(GamepadKeys.Button.A)
                .toggleWhenPressed(
                        new StartEndCommand(
                                launcher::launch,
                                launcher::stop,
                                launcher));
        // Register it so the framework runs its periodic() loop automatically
        register(launcher);

    }

    @Override
    public void runOpMode() throws InterruptedException {
        // 4. Run your initialization blocks
        initialize();

        waitForStart();

        // 5. This is your main robot loop
        while (opModeIsActive() && !isStopRequested()) {
            run(); // <-- This runs all your Subsystem periodic() blocks automatically!
        }

        // Clean up when the match ends
        CommandScheduler.getInstance().reset();
    }
}

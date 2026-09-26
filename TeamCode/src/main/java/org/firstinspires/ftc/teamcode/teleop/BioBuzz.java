package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.StartEndCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

@TeleOp(name = "BioBuzz TeleOp")
public class BioBuzz extends CommandOpMode {

    // 1. Declare your subsystems and controllers here
    private DriveSubsystem driveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private GamepadEx gamePad1;

    @Override
    public void initialize() {
        // 1. Initialize your DriveSubsystem
        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        gamePad1 = new GamepadEx(gamepad1);

        // 2. Set up the default driving command using a RunCommand loop
        driveSubsystem.setDefaultCommand(
                new RunCommand(() -> {
                    // Read inputs (Inverting stick Y because pushing up is natively negative)
                    double drive = -gamepad1.left_stick_y;
                    double turn = gamepad1.right_stick_x;

                    // Pass the inputs straight to your subsystem method
                    driveSubsystem.drive(drive, turn);
                }, driveSubsystem) // Passing driveSubsystem declares it as a requirement
        );

        gamePad1.getGamepadButton(GamepadKeys.Button.B).toggleWhenPressed(
                new StartEndCommand(intakeSubsystem::in,intakeSubsystem::stop,intakeSubsystem)
        );

    }
}

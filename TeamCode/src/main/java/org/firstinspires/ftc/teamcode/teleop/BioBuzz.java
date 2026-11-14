package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.StartEndCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.PushBallSubsystem;

@TeleOp(name = "BioBuzz TeleOp")
public class  BioBuzz extends CommandOpMode {

    // 1. Declare your subsystems and controllers here
    //private DriveSubsystem driveSubsystem;
    private MecanumDriveSubsystem mecanumDriveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private PushBallSubsystem pushBallSubsystem;
    private FlywheelSubsystem flywheelSubsystem;
    private GamepadEx gamePad1;
    private GamepadEx gamePad2;

    @Override
    public void initialize() {
        // 1. Initialize your DriveSubsystem
        mecanumDriveSubsystem = new MecanumDriveSubsystem(hardwareMap, telemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        pushBallSubsystem = new PushBallSubsystem(hardwareMap,"launcher", telemetry);
        flywheelSubsystem = new FlywheelSubsystem(hardwareMap, "launcher", telemetry);
        gamePad1 = new GamepadEx(gamepad1);
        gamePad2 = new GamepadEx(gamepad2);

        // 2. Set up the default driving command using a RunCommand loop
        mecanumDriveSubsystem.setDefaultCommand(
                new RunCommand(() -> {
                    // Read inputs (Inverting stick Y because pushing up is natively negative)
                    double drive = gamepad1.right_stick_y;
                    double turn = gamepad1.left_stick_x;
                    double right = -gamepad1.right_stick_x;

                    // Pass the inputs straight to your subsystem method
                    mecanumDriveSubsystem.driveFieldRelative(drive, right, turn);
                }, mecanumDriveSubsystem) // Passing driveSubsystem declares it as a requirement
        );

        gamePad2.getGamepadButton(GamepadKeys.Button.B).toggleWhenPressed(
                new StartEndCommand(intakeSubsystem::in,intakeSubsystem::stop,intakeSubsystem)
        );
        gamePad2.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(
                new StartEndCommand(flywheelSubsystem::launch,flywheelSubsystem::stop, flywheelSubsystem)
                );
        gamePad2.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(
                new StartEndCommand(pushBallSubsystem::pushBall, pushBallSubsystem::stopServo, pushBallSubsystem)
        );


    }
    @Override
    public void run() {
        gamePad1.readButtons();
        gamePad2.readButtons();
        super.run();
        telemetry.update();
    }
}

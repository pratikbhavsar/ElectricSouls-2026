package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystem.PushBallSubsystem;

import java.util.ArrayList;
import java.util.List;

/**
 * START: back touching your wall, 4 POLLEN loaded, facing the hive, not touching a flower,
 *        not inside the LOADING ZONE.
 *
 * Routine: (optional sidestep under the hive) -> drive out (LEAVE) -> shoot -> strafe into the LOADING ZONE (PARK).
 *
 * Strafe + = right. Red zone is assumed to the robot's RIGHT, blue to the LEFT.
 * If that is backwards for your field, flip ZONE_IS_RIGHT_FOR_RED.
 */
public abstract class BioBuzzAuto extends CommandOpMode {

    // ===================== CALIBRATE THESE =====================
    private static final double MOVE_POWER = 0.6;

    private static final boolean START_FAR_FROM_HIVE = false; // true = sidestep under the hive first
    private static final double  SIDE_TO_HIVE_IN     = 24.0;  // PLACEHOLDER: only if START_FAR_FROM_HIVE

    private static final double LAUNCH_DISTANCE_IN = 20.0;    // PLACEHOLDER: wall to shooting spot
    private static final double ROBOT_DEPTH_IN     = 18.0;    // your robot's length front to back
    private static final double ZONE_CENTER_OUT_IN = 36.0;    // zone middle from the start wall
    private static final double TO_ZONE_IN         = 50.0;    // PLACEHOLDER: sideways distance to the zone

    private static final boolean ZONE_IS_RIGHT_FOR_RED = true;

    private static final long SPINUP_MS = 1500;               // flywheel from rest needs time; tune
    private static final long FEED_MS   = 500;                // continuous servo run time per ball
    private static final long GAP_MS    = 700;                // flywheel recovery between balls
    private static final int  SHOTS     = 2;                  // set to the number that tips the hive

    // Everything before parking must finish by this time so PARK always happens.
    // Time your park leg, then use about 29000 - (park leg time) - margin.
    private static final long PRE_PARK_DEADLINE_MS = 22000;
    // ===========================================================

    private final double zoneSign; // +1 = zone to the right, -1 = left

    private AutoDriveSubsystem driveSubsystem;
    private PushBallSubsystem launcherSubsystem;

    protected BioBuzzAuto(boolean red) {
        this.zoneSign = (red == ZONE_IS_RIGHT_FOR_RED) ? 1.0 : -1.0;
    }

    @Override
    public void initialize() {
        driveSubsystem = new AutoDriveSubsystem(hardwareMap, telemetry);
        launcherSubsystem = new PushBallSubsystem(hardwareMap, "launcher", telemetry);

        List<Command> before = new ArrayList<>();

        if (START_FAR_FROM_HIVE) {
            before.add(move(0, -zoneSign * SIDE_TO_HIVE_IN, 4000));
        }

        // Drive out to the launch point (also earns LEAVE)
        before.add(move(LAUNCH_DISTANCE_IN, 0, 5000));

        // Flywheel runs the whole time while the feeder fires
        List<Command> shoot = new ArrayList<>();
        shoot.add(new WaitCommand(SPINUP_MS));
        for (int i = 0; i < SHOTS; i++) {
            shoot.add(new InstantCommand(launcherSubsystem::pushBall));
            shoot.add(new WaitCommand(FEED_MS));
            shoot.add(new InstantCommand(launcherSubsystem::stopServo));
            shoot.add(new WaitCommand(GAP_MS));
        }
        before.add(new ParallelRaceGroup(
                new RunCommand(launcherSubsystem::launch, launcherSubsystem),
                new SequentialCommandGroup(shoot.toArray(new Command[0]))));

        Command beforePark = new SequentialCommandGroup(before.toArray(new Command[0]))
                .withTimeout(PRE_PARK_DEADLINE_MS);

        // Park: back up to line up with the zone depth and strafe in, in one move
        double depthAdjust = (ZONE_CENTER_OUT_IN - ROBOT_DEPTH_IN / 2.0) - LAUNCH_DISTANCE_IN;

        schedule(new SequentialCommandGroup(
                beforePark,
                // These run even if beforePark timed out, so nothing keeps spinning
                new InstantCommand(launcherSubsystem::stopServo),
                new InstantCommand(launcherSubsystem::stop, launcherSubsystem),
                move(depthAdjust, zoneSign * TO_ZONE_IN, 4000)
        ));
    }

    @Override
    public void run() {
        super.run();
        telemetry.update();
    }

    private Command move(double forwardIn, double strafeIn, long timeoutMs) {
        return new MoveCommand(driveSubsystem, forwardIn, strafeIn, MOVE_POWER).withTimeout(timeoutMs);
    }

    /** Encoder move (forward + strafe) with heading hold, until all four wheels reach target. */
    private static class MoveCommand extends CommandBase {
        private final AutoDriveSubsystem drive;
        private final double forwardIn, strafeIn, power;

        MoveCommand(AutoDriveSubsystem drive, double forwardIn, double strafeIn, double power) {
            this.drive = drive;
            this.forwardIn = forwardIn;
            this.strafeIn = strafeIn;
            this.power = power;
            addRequirements(drive);
        }

        @Override public void initialize() { drive.startMove(forwardIn, strafeIn); }
        @Override public void execute() { drive.updateMove(power); }
        @Override public boolean isFinished() { return drive.moveAtTarget(); }
        @Override public void end(boolean interrupted) { drive.stop(); }
    }

    @Autonomous(name = "BioBuzz Auto RED", group = "Autonomous")
    public static class Red extends BioBuzzAuto {
        public Red() { super(true); }
    }

    @Autonomous(name = "BioBuzz Auto BLUE", group = "Autonomous")
    public static class Blue extends BioBuzzAuto {
        public Blue() { super(false); }
    }
}
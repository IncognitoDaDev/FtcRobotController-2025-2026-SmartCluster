package org.firstinspires.ftc.teamcode.opmode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.commands.SequentialCommand;
import com.smartcluster.oracleftc.commands.ThreadedCommandScheduler;
import com.smartcluster.oracleftc.commands.WaitCommand;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;
import com.smartcluster.oracleftc.math.control.PIDController;
import com.smartcluster.oracleftc.math.filters.MovingAverageFilter;
import com.smartcluster.oracleftc.utils.Performance;

import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Storage;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.atomic.AtomicReference;

@SuppressWarnings("Convert2MethodRef")
@Config
@Autonomous
    public class BlueFarAuto extends LinearOpMode {

    private final CommandScheduler scheduler = new CommandScheduler();
    private final MovingAverageFilter loopTimeFilter = new MovingAverageFilter(50);

    private static Action commandToAction(Command c) {
        return new Action() {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    c.init();
                    initialized = true;
                }
                c.update();
                if (c.finished()) {
                    c.end(false);
                    return false;
                }
                return true;
            }
        };
    }

    private static Command actionToCommand(Action a) {
        return new Command() {
            private boolean finished = false;
            @Override
            public void init() { finished = true; }
            @Override
            public void update() {
                TelemetryPacket p = new TelemetryPacket();
                finished = !a.run(p);
            }
            @Override
            public boolean finished() { return finished; }
            @Override
            public void end(boolean interrupted) { super.end(interrupted); }
            @Override
            public Set<Subsystem> requires() { return super.requires(); }
        };
    }

    private final Pose2d startPose = new Pose2d(-13.5, -59, Math.toRadians(-90));
    private final Pose2d endPose = new Pose2d(-36, -50, Math.toRadians(0));

    private final Pose2d shootPose = new Pose2d(-20, -50, Math.toRadians(-63));

    private final Pose2d stack1 = new Pose2d(-27,-33.5, Math.toRadians(180));
    private final Pose2d stack2 = new Pose2d(-27,-10.5, Math.toRadians(180));
//    private final Pose2d stack3 = new Pose2d(-27,12.5, Math.toRadians(180));

    public static double hoodAngle = 0.615, velocityTarget = 3100;

    public VelConstraint slow = (pose2dDual, posePath, v) -> 30;
    public VelConstraint normal = (pose2dDual, posePath, v) -> 50;
//    public static PIDController pidController = new PIDController(0.0055, 0.0000, 0.00014);


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(this,true);

        scheduler.schedule(robot.update());
        Command.run(new SequentialCommand(
                robot.reset()
        ));

        SequentialAction autoAction = new SequentialAction
                (
                commandToAction(
                        new SequentialCommand(
                                robot.cam.scanOrder(),
                                new InstantCommand(() ->
                                {
                                    robot.storage.storage.OuttakeFacing = -1;
                                    Storage.StorageState.Order = robot.cam.getOrder();
                                    Storage.StorageState.Slot[0] = Storage.ArtifactColor.PURPLE;
                                    Storage.StorageState.Slot[1] = Storage.ArtifactColor.PURPLE;
                                    Storage.StorageState.Slot[2] = Storage.ArtifactColor.GREEN;
                                    robot.turret.setTargetVelocity(1000);
                                    robot.turret.hood.setTarget(hoodAngle);
                                })
                        )),

                        robot.drive.actionBuilder(startPose)
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(shootPose, Math.toRadians(180))
                                .build(),

                        commandToAction( new SequentialCommand( // SHOOT PRE-GAME STACK

                                new InstantCommand(() ->  robot.turret.setTargetVelocity(velocityTarget)),

                                robot.storage.sort(0),
                                robot.turret.WaitForRPM(3000),
                                robot.storage.BallToOuttake(),
                                robot.storage.sort(1),
                                robot.turret.WaitForRPM(1000),
                                robot.storage.BallToOuttake(),
                                robot.storage.sort(2),
                                robot.turret.WaitForRPM(1000),
                                robot.storage.BallToOuttake(),

                                new InstantCommand(() -> robot.turret.setTargetVelocity(1000))
                        )),

                        new ParallelAction( // STACK 1 PICK UP
                                robot.drive.actionBuilder(shootPose)
                                        .setTangent(Math.toRadians(135))
                                        .splineToLinearHeading(stack1, Math.toRadians(180))
                                        .build(),
                                commandToAction(robot.storage.intakeMode())
                        ),

                        new RaceAction(
                                robot.drive.actionBuilder(stack1)
                                        .setTangent(Math.toRadians(180))
                                        .splineToConstantHeading(new Vector2d(-65, stack1.position.y), Math.toRadians(180), slow)
                                        .build(),

                                commandToAction(new SequentialCommand(
                                        robot.intake.intake(),
                                        robot.storage.WaitForBall(3, 2500)
                                ))
                        ),

                        commandToAction( new SequentialCommand(
                                new InstantCommand(() ->
                                {
                                    Storage.StorageState.Slot[0] = Storage.ArtifactColor.GREEN;
                                    Storage.StorageState.Slot[1] = Storage.ArtifactColor.PURPLE;
                                    Storage.StorageState.Slot[2] = Storage.ArtifactColor.PURPLE;

                                }),
                                robot.intake.stop(),
                                robot.storage.outtakeMode(-1)
                        )),

                        robot.drive.actionBuilder(new Pose2d(-65, stack1.position.y, Math.toRadians(180)))
                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(shootPose, Math.toRadians(-90), normal)
                                .build(),

                        commandToAction( new SequentialCommand( // SHOOT STACK 1
                                new InstantCommand(() -> robot.turret.setTargetVelocity(velocityTarget)),

                                robot.storage.sort(0),
                                robot.turret.WaitForRPM(3000),
                                robot.storage.BallToOuttake(),
                                robot.storage.sort(1),
                                robot.turret.WaitForRPM(1000),
                                robot.storage.BallToOuttake(),
                                robot.storage.sort(2),
                                robot.turret.WaitForRPM(1000),
                                robot.storage.BallToOuttake(),

                                new InstantCommand(() -> robot.turret.setTargetVelocity(1000))
                        )),

                        new ParallelAction(
                                robot.drive.actionBuilder(shootPose)
                                        .setTangent(Math.toRadians(180))
                                        .splineToLinearHeading(stack2, Math.toRadians(135))
                                        .build(),
                                commandToAction(robot.storage.intakeMode())
                        ),

                        new RaceAction( // STACK 2 PICK UP
                                robot.drive.actionBuilder(stack2)
                                        .setTangent(Math.toRadians(180))
                                        .splineToConstantHeading(new Vector2d(-65, stack2.position.y), Math.toRadians(180), slow)
                                        .build(),

                                commandToAction(new SequentialCommand(
                                        robot.intake.intake(),
                                        robot.storage.WaitForBall(3, 2500)
                                ))
                        ),

                        commandToAction( new SequentialCommand(
                                new InstantCommand(() ->
                                {
                                    Storage.StorageState.Slot[0] = Storage.ArtifactColor.PURPLE;
                                    Storage.StorageState.Slot[1] = Storage.ArtifactColor.GREEN;
                                    Storage.StorageState.Slot[2] = Storage.ArtifactColor.PURPLE;

                                }),
                                robot.intake.stop(),
                                robot.storage.outtakeMode(-1)
                        )),


                        robot.drive.actionBuilder(new Pose2d(-65, stack2.position.y, Math.toRadians(180)))
                                .setTangent(Math.toRadians(-30))
                                .splineToLinearHeading(shootPose, Math.toRadians(-25), normal)
                                .build(),


                        commandToAction( new SequentialCommand( // SHOOT STACK 2

                                new InstantCommand(() -> robot.turret.setTargetVelocity(velocityTarget)),

                                robot.storage.sort(0),
                                robot.turret.WaitForRPM(3000),
                                robot.storage.BallToOuttake(),
                                robot.storage.sort(1),
                                robot.turret.WaitForRPM(1000),
                                robot.storage.BallToOuttake(),
                                robot.storage.sort(2),
                                robot.turret.WaitForRPM(1000),
                                robot.storage.BallToOuttake(),

//                                new InstantCommand(() -> robot.turret.setTargetVelocity(1000))
                                new InstantCommand(() -> robot.turret.enabledVel.set(false))

                        )),

                        //END OF AUTO
                        robot.drive.actionBuilder(shootPose)
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(endPose, Math.toRadians(90))
                                .build()
        );

        waitForStart();

        robot.drive.localizer.setPose(startPose);

        List<LynxModule> lynxModules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule lynxModule : lynxModules)
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        Canvas c = new Canvas();


        autoAction.preview(c);

        boolean running = true;
        while (running && !isStopRequested()) {
            robot.read();

            TelemetryPacket p = new TelemetryPacket();
            p.fieldOverlay().getOperations().addAll(c.getOperations());
            running = autoAction.run(p);
            scheduler.update();

            FtcDashboard.getInstance().sendTelemetryPacket(p);
            telemetry.addData("Current pose", robot.drive.localizer.getPose().value().position);

            telemetry.addData("Order", robot.cam.getOrderString());

            telemetry.addData("Stock [0]", robot.storage.storage.Slot[0]);
            telemetry.addData("Stock [1]", robot.storage.storage.Slot[1]);
            telemetry.addData("Stock [2]", robot.storage.storage.Slot[2]);

//            telemetry.addData("Dex Current", robot.storage.spindexer.getPosition().get(0));
//            telemetry.addData("Dex Target", robot.storage.spindexer.getTarget());

            telemetry.addData("Dex Error", robot.storage.spindexer.getTarget()-robot.storage.spindexer.getPosition().get(0));
            telemetry.addData("Turret Velocity", robot.turret.getCurrentVelocity());

            telemetry.addData("hz", loopTimeFilter.update(1 / (Performance.loopTimeNano() / 1E9)));
            telemetry.update();
        }

        while (opModeIsActive()) {
            robot.read();

            scheduler.update();
            telemetry.update();
        }
    }
}

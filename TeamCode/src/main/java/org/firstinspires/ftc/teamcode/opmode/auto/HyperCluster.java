package org.firstinspires.ftc.teamcode.opmode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
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

@SuppressWarnings("Convert2MethodRef")
@Config
@Autonomous
public class HyperCluster extends LinearOpMode {

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

    private final Pose2d startPose = new Pose2d(45.2, 57.7, Math.toRadians(-143));
    private final Pose2d shootPose = new Pose2d(12, 12, Math.toRadians(-135));
    private final Pose2d stack1 = new Pose2d(24.5,-35, Math.toRadians(0));
    private final Pose2d stack2 = new Pose2d(24.5,-16.5, Math.toRadians(0));
    private final Pose2d stack3 = new Pose2d(24.5,12, Math.toRadians(0));
    private final Pose2d gatePose = new Pose2d(50, -2, Math.toRadians(-70));
    private final Pose2d endPose = new Pose2d(20, -10, Math.toRadians(180));
    public static double hoodAngle = 0.523, velocityTarget = 2600;

    public VelConstraint slow = (pose2dDual, posePath, v) -> 30;
    public VelConstraint normal = (pose2dDual, posePath, v) -> 50;

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
                        new ParallelAction( // START
                                robot.drive.actionBuilder(startPose)
                                        .setTangent(Math.toRadians(-135))
                                        .splineToSplineHeading(shootPose, Math.toRadians(-135))
                                        .build(),
                                commandToAction( new InstantCommand(() ->
                                                {
                                                    robot.storage.storage.OuttakeFacing = -1;
                                                    Storage.StorageState.Slot[0]= Storage.ArtifactColor.PURPLE;
                                                    Storage.StorageState.Slot[1]= Storage.ArtifactColor.PURPLE;
                                                    Storage.StorageState.Slot[2]= Storage.ArtifactColor.GREEN;
                                                    robot.turret.hood.setTarget(hoodAngle);
                                                })
                                        )),

                        commandToAction( // SHOOT - 1
                                new SequentialCommand(
                                        new InstantCommand(() -> robot.turret.setTargetVelocity(velocityTarget)),

                                        robot.turret.WaitForRPM(2500),
                                        robot.storage.BallToOuttake(),
                                        robot.storage.nextBall(),

                                        robot.turret.WaitForRPM(1000),
                                        robot.storage.BallToOuttake(),
                                        robot.storage.nextBall(),

                                        robot.turret.WaitForRPM(1000),
                                        robot.storage.BallToOuttake(),

                                        new InstantCommand(() -> robot.turret.setTargetVelocity(500))
                                )
                        ),

                        new ParallelAction( // STACK 2 PICK UP
                                robot.drive.actionBuilder(shootPose)
                                        .setTangent(Math.toRadians(-60))
                                        .splineToLinearHeading(stack2, Math.toRadians(-60))
                                        .build(),
                                commandToAction(robot.storage.intakeMode())
                        ),

                        new RaceAction(
                                robot.drive.actionBuilder(stack2)
                                        .setTangent(Math.toRadians(0))
                                        .splineToConstantHeading(new Vector2d(57, stack2.position.y), Math.toRadians(0), slow)
                                        .build(),

                                commandToAction(new SequentialCommand(
                                                robot.intake.intake(),
                                                robot.storage.WaitForBall(3, 3500)
                                        ))
                        ),

                        commandToAction( new SequentialCommand(
                                new InstantCommand(() ->
                                {
                                    Storage.StorageState.Slot[0]= Storage.ArtifactColor.PURPLE;
                                    Storage.StorageState.Slot[1]= Storage.ArtifactColor.PURPLE;
                                    Storage.StorageState.Slot[2]= Storage.ArtifactColor.GREEN;

                                }),
                                robot.intake.stop(),
                                robot.storage.outtakeMode(-1)
                        )),


                        new SequentialAction( // GATE RELEASE !!!
                                new RaceAction(
                                        robot.drive.actionBuilder(new Pose2d(57, 12, Math.toRadians(0)))
                                                .setTangent(Math.toRadians(230))
                                                .splineToLinearHeading(gatePose, Math.toRadians(95), normal)
                                                .build(),
                                        commandToAction(new WaitCommand(3500))
                                ),
                                commandToAction( new SequentialCommand(
                                        robot.cam.scanOrder(),
                                        new InstantCommand(()-> Storage.StorageState.Order = robot.cam.getOrder())
                                ))
                        ),

                        robot.drive.actionBuilder(gatePose)
                                .setTangent(Math.toRadians(-150))
                                .splineToLinearHeading(shootPose, Math.toRadians(-190), normal)
                                .build(),

                        commandToAction( new SequentialCommand(
                                new InstantCommand(() -> robot.turret.setTargetVelocity(velocityTarget)),

                                robot.storage.sort(0),
                                robot.turret.WaitForRPM(2500),
                                robot.storage.BallToOuttake(),
                                robot.storage.sort(1),
                                robot.turret.WaitForRPM(1000),
                                robot.storage.BallToOuttake(),
                                robot.storage.sort(2),
                                robot.turret.WaitForRPM(1000),
                                robot.storage.BallToOuttake(),

                                new InstantCommand(() -> robot.turret.setTargetVelocity(500))
                        )),

                        new ParallelAction(
                                robot.drive.actionBuilder(shootPose)
                                        .setTangent(Math.toRadians(15))
                                        .splineToLinearHeading(stack3, Math.toRadians(0))
                                        .build(),
                                commandToAction(robot.storage.intakeMode())
                        ),

                        new RaceAction(
                                robot.drive.actionBuilder(stack3)
                                        .setTangent(Math.toRadians(0))
                                        .splineToConstantHeading(new Vector2d(51, stack3.position.y), Math.toRadians(0), slow)
                                        .build(),

                                commandToAction(new SequentialCommand(
                                        robot.intake.intake(),
                                        robot.storage.WaitForBall(3, 3500)
                                ))),

                        commandToAction( new SequentialCommand(
                                new InstantCommand(() ->
                                {
                                    Storage.StorageState.Slot[0]= Storage.ArtifactColor.PURPLE;
                                    Storage.StorageState.Slot[1]= Storage.ArtifactColor.PURPLE;
                                    Storage.StorageState.Slot[2]= Storage.ArtifactColor.GREEN;

                                }),
                                robot.intake.stop(),
                                robot.storage.outtakeMode(-1)
                        )),

                        robot.drive.actionBuilder(new Pose2d(51, stack3.position.y, Math.toRadians(0)))
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(shootPose, Math.toRadians(180), normal)
                                .build(),

                        commandToAction( new SequentialCommand( // SHOOT THIRD
                                new InstantCommand(() -> robot.turret.setTargetVelocity(velocityTarget)),

                                robot.storage.sort(0),
                                robot.turret.WaitForRPM(2500),
                                robot.storage.BallToOuttake(),
                                robot.storage.sort(1),
                                robot.turret.WaitForRPM(1000),
                                robot.storage.BallToOuttake(),
                                robot.storage.sort(2),
                                robot.turret.WaitForRPM(1000),
                                robot.storage.BallToOuttake(),

                                new InstantCommand(() -> robot.turret.enabledVel.set(false))
                                )),

                        //END OF AUTO
                        robot.drive.actionBuilder(shootPose)
                                .setTangent(Math.toRadians(-50))
                                .splineToLinearHeading(endPose, Math.toRadians(-50))
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
            robot.drive.updatePoseEstimate();
            scheduler.update();

            FtcDashboard.getInstance().sendTelemetryPacket(p);
            telemetry.addData("Current pose", robot.drive.localizer.getPose().value().position);

            telemetry.addData("Order", robot.cam.getOrderString());

            telemetry.addData("Stock [0]", Storage.StorageState.Slot[0]);
            telemetry.addData("Stock [1]", Storage.StorageState.Slot[1]);
            telemetry.addData("Stock [2]", Storage.StorageState.Slot[2]);

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

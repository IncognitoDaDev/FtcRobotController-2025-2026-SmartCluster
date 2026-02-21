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
    private final Pose2d shootPose = new Pose2d(12, 12,Math.toRadians(-135));
    private final Pose2d stack1 = new Pose2d(28,-35,Math.toRadians(0));
    private final Pose2d stack2 = new Pose2d(28,-10.5,Math.toRadians(0));
    private final Pose2d stack3 = new Pose2d(28,12,Math.toRadians(0));
    private final Pose2d gatePose = new Pose2d(52, -1, Math.toRadians(-90));
    private final Pose2d endPose = new Pose2d(24, -15, Math.toRadians(-90));
    public static double hoodAngle = 0.4;

    public VelConstraint gate = (pose2dDual, posePath, v) -> 25;

    public VelConstraint slow = (pose2dDual, posePath, v) -> 30;
    public VelConstraint normal = (pose2dDual, posePath, v) -> 60;

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
                        new ParallelAction(
                                robot.drive.actionBuilder(startPose)
                                        .setTangent(Math.toRadians(-135))
                                        .splineToSplineHeading(shootPose, Math.toRadians(-135))
//                        .turnTo(Math.toRadians(270+30))
                                        .build(),
                                commandToAction(
                                                new InstantCommand(() ->
                                                {
                                                    robot.storage.storage.OuttakeFacing = -1;
                                                    Storage.StorageState.Slot[0]= Storage.ArtifactColor.PURPLE;
                                                    Storage.StorageState.Slot[1]= Storage.ArtifactColor.PURPLE;
                                                    Storage.StorageState.Slot[2]= Storage.ArtifactColor.GREEN;

                                                })
                                        )),
                        commandToAction(
                                new SequentialCommand(
                                        new InstantCommand(()->robot.turret.setTargetVelocity(2700)),
                                        new InstantCommand(()->robot.turret.hood.setTarget(0.4)),
                                        robot.turret.WaitForRPM(2000),
                                        robot.storage.BallToOuttake(),
                                        robot.storage.nextBall(),

                                        new InstantCommand(()->robot.turret.hood.setTarget(0.41)),
                                        robot.turret.WaitForRPM(500),
                                        robot.storage.BallToOuttake(),
                                        robot.storage.nextBall(),

                                        new InstantCommand(()->robot.turret.hood.setTarget(0.41)),
                                        robot.turret.WaitForRPM(500),
                                        robot.storage.BallToOuttake()
                                )
                        ),
                        //End of pre-load, Stack 2, Then gate
                        new ParallelAction(
                                robot.drive.actionBuilder(shootPose)
                                        .setTangent(Math.toRadians(-60))
                                        .splineToLinearHeading(stack2,Math.toRadians(-60))
                                        .build(),
                                commandToAction(robot.storage.intakeMode())
                        ),
                        new ParallelAction(
                                robot.drive.actionBuilder(stack2)
                                        .setTangent(Math.toRadians(0))
                                        .splineToConstantHeading(new Vector2d(56, -12.5), Math.toRadians(0),slow)
                                        .build(),
                                commandToAction(
                                        new SequentialCommand(
                                                robot.intake.intake(),
                                                robot.storage.WaitForBall(3, 3200),
                                                //De schimbat👍
                                                new InstantCommand(() ->
                                                {
                                                    Storage.StorageState.Slot[0]= Storage.ArtifactColor.PURPLE;
                                                    Storage.StorageState.Slot[1]= Storage.ArtifactColor.GREEN;
                                                    Storage.StorageState.Slot[2]= Storage.ArtifactColor.PURPLE;

                                                }),
                                                robot.intake.stop(),
                                                robot.storage.outtakeMode(-1)
                                        ))
                        ),
                        //INNER GATE 8, RELEASE
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(56, 12, Math.toRadians(0)))
                                    .setTangent(Math.toRadians(180))
                                    .splineToLinearHeading(gatePose, Math.toRadians(180),normal)
                                    .build(),
                                commandToAction(
                                        new SequentialCommand(
                                                //Sizeable amount, based on gate timing
                                            new WaitCommand(500),
                                            robot.cam.scanOrder(),
                                            new InstantCommand(()-> Storage.StorageState.Order = robot.cam.getOrder())

                                    )
                                )
                        ),
                        robot.drive.actionBuilder(gatePose)
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(shootPose, Math.toRadians(180),normal)
                                .build(),
                        commandToAction(
                                new SequentialCommand(
                                        new InstantCommand(() -> {
                                            robot.turret.setTargetVelocity(2700);
                                            robot.turret.hood.setTarget(hoodAngle);
                                        }),

                                        robot.storage.sort(0),
                                        robot.turret.WaitForRPM(2000),
                                        robot.storage.BallToOuttake(),
                                        robot.storage.sort(1),
                                        robot.turret.WaitForRPM(1000),
                                        robot.storage.BallToOuttake(),
                                        robot.storage.sort(2),
                                        robot.turret.WaitForRPM(1000),
                                        robot.storage.BallToOuttake(),

                                        new InstantCommand(() -> robot.turret.setTargetVelocity(500))
                                )

                        ),
                        //End of second shoot, Stack 1
                        new ParallelAction(
                                robot.drive.actionBuilder(shootPose)
                                        .setTangent(Math.toRadians(-90))
                                        .splineToLinearHeading(stack3,Math.toRadians(0))
                                        .build(),
                                commandToAction(robot.storage.intakeMode())
                        ),
                        new ParallelAction(
                                robot.drive.actionBuilder(stack3)
                                        .setTangent(Math.toRadians(180))
                                        .splineToConstantHeading(new Vector2d(53, stack3.position.y), Math.toRadians(180), slow)
                                        .build(),

                                commandToAction(new SequentialCommand(
                                        robot.intake.intake(),
                                        robot.storage.distanceSwitch(3, 3200),
                                        new InstantCommand(() ->
                                        {
                                            Storage.StorageState.Slot[0]= Storage.ArtifactColor.PURPLE;
                                            Storage.StorageState.Slot[1]= Storage.ArtifactColor.PURPLE;
                                            Storage.StorageState.Slot[2]= Storage.ArtifactColor.GREEN;

                                        }),
                                        robot.intake.stop(),
                                        robot.storage.outtakeMode(-1)
                                ))),
                        //Third shoot
                        robot.drive.actionBuilder(new Pose2d(60, 12, Math.toRadians(0)))
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(shootPose, Math.toRadians(180),normal)
                                .build(),
                        commandToAction(
                                new SequentialCommand(
                                        new InstantCommand(() -> {
                                            robot.turret.setTargetVelocity(2700);
                                            robot.turret.hood.setTarget(hoodAngle);
                                        }),

                                        robot.storage.sort(0),
                                        robot.turret.WaitForRPM(2000),
                                        robot.storage.BallToOuttake(),
                                        robot.storage.sort(1),
                                        robot.turret.WaitForRPM(1000),
                                        robot.storage.BallToOuttake(),
                                        robot.storage.sort(2),
                                        robot.turret.WaitForRPM(1000),
                                        robot.storage.BallToOuttake(),

                                        new InstantCommand(() -> robot.turret.setTargetVelocity(500))
                                )

                        ),
                        //End of third shoot, stack 2
                        robot.drive.actionBuilder(shootPose)
                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(endPose,Math.toRadians(0))
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

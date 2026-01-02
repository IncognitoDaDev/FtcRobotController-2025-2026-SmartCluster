package org.firstinspires.ftc.teamcode.opmode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import com.smartcluster.oracleftc.commands.Command;
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
import java.util.concurrent.atomic.AtomicReference;

@SuppressWarnings("Convert2MethodRef")

@Autonomous
    public class BlueFarAuto extends LinearOpMode {

    private final ThreadedCommandScheduler scheduler = new ThreadedCommandScheduler();
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

    private final Pose2d startPose = new Pose2d(-13, -62, Math.toRadians(270));
    private final Pose2d endPose = new Pose2d(-15, -56, Math.toRadians(295));
    private final Pose2d blueCorner = new Pose2d(60, 63, -45);

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Robot robot = new Robot(this);

        scheduler.schedule(robot.update());

        SequentialAction autoAction = new SequentialAction
        (
                commandToAction(
                        new SequentialCommand(
                                robot.cam.scanOrder(),
                                new InstantCommand(() ->
                                {
                                    robot.storage.storage.OuttakeFacing = -1;
                                    robot.storage.storage.Order = robot.cam.getOrder();
                                })
                        )),

                robot.drive.actionBuilder(startPose)
                        .splineToConstantHeading(new Vector2d(-15, -56), Math.toRadians(0))
                        .turnTo(Math.toRadians(270+25))
                        .build(),

                commandToAction(
                        new SequentialCommand(
                                robot.storage.routineBallCheck(), // caches the ball into data
                                robot.storage.outtakeMode(-1),

                                new InstantCommand(() -> {
                                    robot.turret.setTargetVelocity(4400);
                                    robot.turret.hood.setTarget(0.40);
                                }),

                                robot.storage.sort(0),
                                robot.turret.WaitForRPM(2000),
                                robot.storage.BallToOuttake(),
                                //new InstantCommand(()->robot.turret.hood.setTarget(0.42)),
                                robot.storage.sort(1),
                                robot.turret.WaitForRPM(1000),
                                robot.storage.BallToOuttake(),
                                //new InstantCommand(()->robot.turret.hood.setTarget(0.44)),
                                robot.storage.sort(2),
                                robot.turret.WaitForRPM(1000),
                                robot.storage.BallToOuttake(),

                                new InstantCommand(() -> robot.turret.setTargetVelocity(0))
                        )
                ),

                commandToAction(robot.reset())
        );

        waitForStart();

        Command.run(robot.reset());
        robot.drive.localizer.setPose(startPose);
        Thread.sleep(100);

        List<LynxModule> lynxModules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule lynxModule : lynxModules)
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        Canvas c = new Canvas();
        ExecutorService pool = Executors.newFixedThreadPool(2);
        List<Callable<Void>> callables = new ArrayList<>();
        callables.add(() -> { robot.drive.localizer.update(); return null; });
        for (LynxModule lynxModule : lynxModules) {
                callables.add(() -> { lynxModule.clearBulkCache(); lynxModule.getBulkData(); return null; });
        }

        autoAction.preview(c);

        boolean running = true;
        while (running && !Thread.currentThread().isInterrupted()) {
            for (Future<Void> future : pool.invokeAll(callables)) {
                try { future.get(); } catch (ExecutionException e) { RobotLog.logStackTrace(e); }
            }
            TelemetryPacket p = new TelemetryPacket();
            p.fieldOverlay().getOperations().addAll(c.getOperations());
            running = autoAction.run(p);
            scheduler.update();

//            Storage.ArtifactColor[] order = robot.cam.getOrder();
            FtcDashboard.getInstance().sendTelemetryPacket(p);
//            telemetry.addData("Order [0]", robot.storage.storage.Order[0]);
//            telemetry.addData("Order [1]", robot.storage.storage.Order[1]);
//            telemetry.addData("Order [2]", robot.storage.storage.Order[2]);
            telemetry.addData("Order [0]", robot.cam.getOrder()[0]);
            telemetry.addData("Order [1]", robot.cam.getOrder()[1]);
            telemetry.addData("Order [2]", robot.cam.getOrder()[2]);
            telemetry.addData("Stock [0]", robot.storage.storage.Slot[0]);
            telemetry.addData("Stock [1]", robot.storage.storage.Slot[1]);
            telemetry.addData("Stock [2]", robot.storage.storage.Slot[2]);

            telemetry.addData("hz", loopTimeFilter.update(1 / (Performance.loopTimeNano() / 1E9)));
            telemetry.update();
        }

        while (opModeIsActive()) {
            for (LynxModule lynxModule : lynxModules)
            {
                    lynxModule.clearBulkCache();
                    lynxModule.getBulkData();
            }

            scheduler.update();
            telemetry.update();
        }
    }
}

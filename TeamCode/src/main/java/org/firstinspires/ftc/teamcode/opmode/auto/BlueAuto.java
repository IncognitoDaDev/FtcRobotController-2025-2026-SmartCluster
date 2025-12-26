package org.firstinspires.ftc.teamcode.opmode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.ThreadedCommandScheduler;
import com.smartcluster.oracleftc.hardware.OracleOptimize;
import com.smartcluster.oracleftc.math.filters.MovingAverageFilter;
import com.smartcluster.oracleftc.utils.Performance;

import org.firstinspires.ftc.teamcode.subsystem.Robot;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

@OracleOptimize
@Autonomous
public class BlueAuto extends LinearOpMode {

    private final ThreadedCommandScheduler scheduler = new ThreadedCommandScheduler();

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
                } else return true;
            }
        };
    }

    public static final Pose2d startPose = new Pose2d(11.3 ,-63,Math.toRadians(90));
    public final Pose2d ShootPose = new Pose2d(13.5, 60, Math.toRadians(200));
    private static Pose2d stack1= new Pose2d(31.3,37.3, Math.toRadians(0));




    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(this);

        Command.run(robot.reset());

        scheduler.schedule(robot.update());

        robot.drive.localizer.setPose(startPose);
        MovingAverageFilter loopTimeFilter=new MovingAverageFilter(50);
        Action autoAction = new SequentialAction(
                robot.drive.actionBuilder(startPose)
                        .lineToYConstantHeading(40)
                        .build()
//                commandToAction(
//                        new SequentialCommand(
//                                new ParallelCommand(
//                                        new InstantCommand(()-> {
//                                            robot.turret.hood.setTarget(0.7);
//                                            robot.turret.setShooterSpeed(5400);
//                                            robot.turret.setAngle(0.0);
//                                        })
//
//                                ),
//                                new WaitCommand(200),
//                                new InstantCommand(robot.spindex::FlapperUp),
//                                new WaitCommand(200),
//                                robot.spindex.NextSpace(),
//                                new InstantCommand(robot.spindex::FlapperUp),
//                                new WaitCommand(200),
//                                robot.spindex.NextSpace(),
//                                new InstantCommand(robot.spindex::FlapperUp),
//                                new WaitCommand(200),
//                                robot.spindex.NextSpace(),
//                                new InstantCommand(()->robot.turret.hood.setTarget(0.7)),
//                                new InstantCommand(() ->
//                                        robot.mecanumDrive.actionBuilder(startPose)
//                                                .splineToSplineHeading(stack1, 0)
//                                                .build()),
//
//                                new RaceCommand(
//                                        new ParallelCommand(
//                                                new InstantCommand(() ->
//                                                        robot.mecanumDrive.actionBuilder(stack1)
//                                                                .lineToX(53)
//                                                                .build()
//                                                ),
//                                                new SequentialCommand(
//                                                        new InstantCommand(robot.intake::intake),
//                                                        new WaitCommand(200),
//                                                        robot.spindex.NextSpace(),
//                                                        new WaitCommand(200),
//                                                        robot.spindex.NextSpace(),
//                                                        new WaitCommand(150),
//                                                        new InstantCommand(robot.intake::reset)
//                                                )
//                                        ),
//                                        new WaitCommand(4000)
//
//                                )
//
//                        )
                );


        waitForStart();

        Thread.sleep(100);
        List<LynxModule> lynxModules = hardwareMap.getAll(LynxModule.class);
        for(LynxModule lynxModule: lynxModules)
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        Canvas c = new com.acmerobotics.dashboard.canvas.Canvas();
        ExecutorService pool = Executors.newFixedThreadPool(2);
        List<Callable<Void>>  callables = new ArrayList<>();
        callables.add(()->{robot.drive.localizer.update(); return null;});
        for(LynxModule lynxModule: lynxModules)
            if(lynxModule.getSerialNumber().isEmbedded())
            {
                callables.add(()->{
                    lynxModule.clearBulkCache();
                    lynxModule.getBulkData();
                    return null;
                });
            }
        autoAction.preview(c);
        boolean b = true;
        while (b && !Thread.currentThread().isInterrupted()) {
            for (Future<Void> future : pool.invokeAll(callables)) {
                try {
                    future.get();
                } catch (ExecutionException e) {
                    RobotLog.logStackTrace(e);
                }
            }
            TelemetryPacket p = new TelemetryPacket();
            p.fieldOverlay().getOperations().addAll(c.getOperations());

            b = autoAction.run(p);
            scheduler.update();
            FtcDashboard.getInstance().sendTelemetryPacket(p);
            telemetry.addData("hz", loopTimeFilter.update(1/(Performance.loopTimeNano()/1E9)));
            telemetry.update();
        }
        while(opModeIsActive())
        {
            for(LynxModule lynxModule: lynxModules)
                if(lynxModule.getSerialNumber().isEmbedded())
                {
                    lynxModule.clearBulkCache();
                    lynxModule.getBulkData();

                }
            scheduler.update();
            telemetry.addData("hz", loopTimeFilter.update(1/(Performance.loopTimeNano()/1E9)));
            telemetry.update();

        }
    }
}
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
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.commands.ParallelCommand;
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
@Config
@Autonomous
public class BlueCloseAuto extends LinearOpMode {

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

    private final Pose2d startPose = new Pose2d(-46.5, 55.5, Math.toRadians(-125));
    private final Pose2d shootPose = new Pose2d(-12, 12,Math.toRadians(180));
    private final Vector2d shootpose = new Vector2d(-12,12);
    private final Pose2d stack1 = new Pose2d(-28,-35,Math.toRadians(180));
    private final Pose2d stack2 = new Pose2d(-28,-11.5,Math.toRadians(180));
    private final Pose2d stack3 = new Pose2d(-28,12.5,Math.toRadians(180));

    private final Pose2d endPose = new Pose2d(-24, -15, Math.toRadians(-90));
    public static double hoodAngle = 0.58;
    public VelConstraint slow = (pose2dDual, posePath, v) -> 20;
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
                        //FIRST SHOOT
                        new ParallelAction( // Move to a better pos for obelisk scan
                                robot.drive.actionBuilder(startPose)
                                        .setTangent(Math.toRadians(-45))
                                        .splineToLinearHeading(new Pose2d(shootpose,Math.toRadians(90)), Math.toRadians(-45))
                                        .build(),

                                commandToAction(
                                        new SequentialCommand(
                                            new WaitCommand(1500),
                                            robot.cam.scanOrder(),
                                            new InstantCommand(() ->
                                            {
                                                robot.storage.storage.OuttakeFacing = -1;
                                                Storage.StorageState.Order = robot.cam.getOrder();
                                                robot.storage.storage.Slot[0]= Storage.ArtifactColor.PURPLE;
                                                robot.storage.storage.Slot[1]= Storage.ArtifactColor.PURPLE;
                                                robot.storage.storage.Slot[2]= Storage.ArtifactColor.GREEN;
                                            })
                                ))),
                        robot.drive.actionBuilder(startPose)
                                    .turnTo(Math.toRadians(-45))
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
                        )),
                        //STACK 3 INTAKE
                        new ParallelAction( //Heads to stack3 pos
                                robot.drive.actionBuilder(shootPose)
                                        .setTangent(Math.toRadians(180))
                                        .splineToLinearHeading(stack3, Math.toRadians(180))
                                        .build(),
                                commandToAction(robot.storage.intakeMode())
                        ),

                        new ParallelAction( // Intakes stack3
                                robot.drive.actionBuilder(stack3)
                                        .setTangent(Math.toRadians(180))
                                        .splineToConstantHeading(new Vector2d(-54, stack3.position.y), Math.toRadians(180), slow)
                                        .build(),

                                commandToAction(new SequentialCommand(
                                        robot.intake.intake(),
                                        robot.storage.WaitForBall(3,3500),
                                        robot.intake.stop()
                                ))
                        ),
                        //SECOND SHOOT
                        new ParallelAction( //Heads to shooting pos (from stack3)
                                robot.drive.actionBuilder(new Pose2d(-54, stack3.position.y, Math.toRadians(0)))
                                        .setTangent(Math.toRadians(0))
                                        .splineToConstantHeading(shootpose, Math.toRadians(0), normal)
                                        .build(),
                                commandToAction(robot.storage.outtakeMode(-1))
                        ),

                        commandToAction( new SequentialCommand(
                                new InstantCommand(() -> {
                                    robot.turret.setTargetVelocity(2700);
                                    robot.turret.hood.setTarget(hoodAngle);
                                }),

                                robot.storage.sort(0),
                                robot.turret.WaitForRPM(1000),
                                robot.storage.BallToOuttake(),
                                robot.storage.sort(1),
                                robot.turret.WaitForRPM(1000),
                                robot.storage.BallToOuttake(),
                                robot.storage.sort(2),
                                robot.turret.WaitForRPM(1000),
                                robot.storage.BallToOuttake(),

                                new InstantCommand(() -> robot.turret.setTargetVelocity(0))
                        )),
//
//                        //SECOND STACK
                        new ParallelAction(
                        robot.drive.actionBuilder(shootPose)
                                .setTangent(Math.toRadians(-100))
                                .splineToLinearHeading(stack2, Math.toRadians(180))
                                .build(),
                        commandToAction(robot.storage.intakeMode())
                        ),

                        new ParallelAction(
                                robot.drive.actionBuilder(stack2)
                                        .setTangent(Math.toRadians(180))
                                        .splineToConstantHeading(new Vector2d(-50, stack2.position.y), Math.toRadians(180), slow)
                                        .build(),

                                commandToAction(new SequentialCommand(
                                        robot.intake.intake(),
                                        robot.storage.WaitForBall(3,3500),
                                        robot.intake.stop()
                                ))
                        ),
                        //THIRD SHOOT
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(-54, stack2.position.y, Math.toRadians(0)))
                                        .setTangent(Math.toRadians(0))
                                        .splineToLinearHeading(shootPose, Math.toRadians(45), normal)
                                        .build(),
                                commandToAction(robot.storage.outtakeMode(-1))
                        ),

                        commandToAction( new SequentialCommand(
                                new InstantCommand(() -> {
                                    robot.turret.setTargetVelocity(2700);
                                    robot.turret.hood.setTarget(hoodAngle);
                                }),

                                robot.storage.sort(0),
                                robot.turret.WaitForRPM(1000),
                                robot.storage.BallToOuttake(),
                                robot.storage.sort(1),
                                robot.turret.WaitForRPM(1000),
                                robot.storage.BallToOuttake(),
                                robot.storage.sort(2),
                                robot.turret.WaitForRPM(1000),
                                robot.storage.BallToOuttake(),

                                new InstantCommand(() -> robot.turret.setTargetVelocity(0))
                        )),

                        robot.drive.actionBuilder(shootPose)
                                .setTangent(Math.toRadians(-45))
                                .splineToLinearHeading(endPose, Math.toRadians(-45))
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
            robot.powerManager.read();

            TelemetryPacket p = new TelemetryPacket();
            p.fieldOverlay().getOperations().addAll(c.getOperations());
            robot.drive.updatePoseEstimate();
            running = autoAction.run(p);
            scheduler.update();


            FtcDashboard.getInstance().sendTelemetryPacket(p);
            telemetry.addData("Order [0]", robot.cam.getOrder()[0]);
            telemetry.addData("Order [1]", robot.cam.getOrder()[1]);
            telemetry.addData("Order [2]", robot.cam.getOrder()[2]);
            telemetry.addData("Stock [0]", robot.storage.storage.Slot[0]);
            telemetry.addData("Stock [1]", robot.storage.storage.Slot[1]);
            telemetry.addData("Stock [2]", robot.storage.storage.Slot[2]);
            telemetry.addData("Flywheel speed", robot.turret.getCurrentVelocity());
            telemetry.addData("Hood angle", hoodAngle);

            telemetry.addData("hz", loopTimeFilter.update(1 / (Performance.loopTimeNano() / 1E9)));
            telemetry.update();
        }

        while (opModeIsActive()) {
            robot.powerManager.read();

            scheduler.update();
            telemetry.update();
        }
    }
}

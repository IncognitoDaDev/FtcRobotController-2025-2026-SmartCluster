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
public class SPINDEX extends LinearOpMode {

    private final CommandScheduler scheduler = new CommandScheduler();
    private final MovingAverageFilter loopTimeFilter = new MovingAverageFilter(50);
    public static PIDController pidController = new PIDController(0.0043, 0.000078, 0.00007);

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
    private final Pose2d startPose = new Pose2d(13,-59, Math.toRadians(-90));
    private final Pose2d shootPose = new Pose2d(15,-54,Math.toRadians(-122));

    private final Pose2d stack1 = new Pose2d(28.5,-35.5,Math.toRadians(0));
    private final Pose2d stack2 = new Pose2d(28.5,-11,Math.toRadians(0));
    private final Pose2d stack3 = new Pose2d(28,12.5,Math.toRadians(0));

    private final Pose2d endPose = new Pose2d(35, -56, Math.toRadians(90));
    public static double hoodAngle = 0.62;


    public VelConstraint slow = (pose2dDual, posePath, v) -> 25;
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
                        commandToAction(
                                new SequentialCommand(
//                                        robot.cam.scanOrder(),
                                        new InstantCommand(()->robot.storage.PID_Selector(pidController.p,pidController.i,pidController.d)),
                                        new InstantCommand(() ->
                                        {
                                            robot.storage.storage.OuttakeFacing = -1;
                                            Storage.StorageState.Order = robot.cam.getOrder();
                                            robot.storage.storage.Slot[0]= Storage.ArtifactColor.PURPLE;
                                            robot.storage.storage.Slot[1]= Storage.ArtifactColor.PURPLE;
                                            robot.storage.storage.Slot[2]= Storage.ArtifactColor.GREEN;

                                        })
                                )),
                        commandToAction(
                                new SequentialCommand(
                                        new InstantCommand(()->robot.storage.PID_Selector(pidController.p,pidController.i,pidController.d)),
                                        robot.cam.scanOrder(),

                                            robot.storage.nextBall(),
                                        new WaitCommand(2000),

                                        robot.storage.nextBall(),
                                        new WaitCommand(2000),
                                        robot.storage.nextBall(),
                                        new WaitCommand(2000),
                                        robot.storage.nextBall(),
                                        new WaitCommand(2000),

                                        robot.storage.nextBall(),
                                        new WaitCommand(2000),

                                        robot.storage.nextBall(),
                                        new WaitCommand(2000),

                                        robot.storage.nextBall(),
                                        new WaitCommand(2000),

                                        robot.storage.nextBall(),
                                        new WaitCommand(2000),

                                        robot.storage.nextBall(),
                                        new WaitCommand(2000),
                                        robot.storage.nextBall(),
                                        new WaitCommand(2000),

                                        robot.storage.nextBall(),
                                        new WaitCommand(2000),

                                        robot.storage.nextBall(),
                                        new WaitCommand(2000),

                                        robot.storage.nextBall(),
                                        new WaitCommand(2000)


                                        ))

                        //HYPERRION EMAIL- hyperion.cnme@gmail.com
                        //HYPERRION EMAIL- hyperion.cnme@gmail.com
                        //HYPERRION EMAIL- hyperion.cnme@gmail.com
                        //HYPERRION EMAIL- hyperion.cnme@gmail.com
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
            for (LynxModule lynxModule : lynxModules)
            {
                lynxModule.clearBulkCache();
                lynxModule.getBulkData();
            }
            TelemetryPacket p = new TelemetryPacket();
            p.fieldOverlay().getOperations().addAll(c.getOperations());
//            robot.drive.updatePoseEstimate();
            scheduler.update();
            running = autoAction.run(p);


//            Storage.ArtifactColor[] order = robot.cam.getOrder();
            FtcDashboard.getInstance().sendTelemetryPacket(p);
            telemetry.addData("CAM reading", robot.cam.getOrder());
            telemetry.addData("Current pose", robot.drive.localizer.getPose().position);
            telemetry.addData("Order [0]", robot.cam.getOrder()[0]);
            telemetry.addData("Order [1]", robot.cam.getOrder()[1]);
            telemetry.addData("Order [2]", robot.cam.getOrder()[2]);
            telemetry.addData("Stock [0]", robot.storage.storage.Slot[0]);
            telemetry.addData("Stock [1]", robot.storage.storage.Slot[1]);
            telemetry.addData("Stock [2]", robot.storage.storage.Slot[2]);
            telemetry.addData("Flywheel speed", robot.turret.getCurrentVelocity());
            telemetry.addData("Hood angle", hoodAngle);
            telemetry.addData("DEX",robot.storage.spindexer.getPosition());
            telemetry.addData("DEX target",robot.storage.spindexer.getTarget());


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

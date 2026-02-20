package org.firstinspires.ftc.teamcode.calibration;

//import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;
//import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.commands.SequentialCommand;
import com.smartcluster.oracleftc.fsm.FSM;
import com.smartcluster.oracleftc.math.control.TrapezoidalMotionProfile;
import com.smartcluster.oracleftc.math.filters.MovingAverageFilter;
import com.smartcluster.oracleftc.utils.Performance;
import com.smartcluster.oracleftc.utils.ProcessedGamepad;

import org.firstinspires.ftc.teamcode.opmode.teleop.BaseTeleOp;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

import java.util.List;

@Config
@TeleOp(group="Calibration")
public class turretCalibration extends LinearOpMode {
    public static TrapezoidalMotionProfile mp = new TrapezoidalMotionProfile(6000, 10000, 12000);
    private final CommandScheduler scheduler = new CommandScheduler();
    private static Pose2d currentPose;
    public static double velocity = 0;
    public static double angle = 0;

    public enum TeleOpState {
        INIT, Tracking
    }

    public static double Kv = 1.1;
    public static double Ka = 0.2;
    public static double Ks = 0.001;
//    FeedforwardCoefficients coefficients = new FeedforwardCoefficients(Kv, Ka, Ks);
//    BasicFeedforward shootController = new BasicFeedforward(coefficients);

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ElapsedTime resetTime = new ElapsedTime();

        Robot robot = new Robot(this,true);
        ProcessedGamepad driverGamepad = new ProcessedGamepad(gamepad1),
                operatorGamepad = new ProcessedGamepad(gamepad2);
        waitForStart();

        List<LynxModule> lynxModules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule lynxModule : lynxModules)
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        scheduler.schedule(
               robot.update()
        );
        FSM.FSMBuilder<BaseTeleOp.TeleOpState> fsmBuilder =  FSM.<BaseTeleOp.TeleOpState>builder()
                .initial(BaseTeleOp.TeleOpState.INIT)
                .transition(BaseTeleOp.TeleOpState.INIT, BaseTeleOp.TeleOpState.IDLE,this::opModeIsActive,
                        new SequentialCommand(
                                robot.storage.spindexer.reset(),
                                new InstantCommand(()->robot.drive.localizer.setPose(new Pose2d(0,0,0))),
                                robot.reset(),
                                robot.turret.reset()
                        ))
                .transition(BaseTeleOp.TeleOpState.IDLE, BaseTeleOp.TeleOpState.IDLE,driverGamepad.cross.pressed(),
                            new SequentialCommand(
                            robot.turret.WaitForRPM(1000),
                            robot.storage.flapperUp()
                ))
                .transition(BaseTeleOp.TeleOpState.IDLE, BaseTeleOp.TeleOpState.IDLE,driverGamepad.cross.released(),
                        robot.storage.flapperDown()
                )
                .transition(BaseTeleOp.TeleOpState.IDLE, BaseTeleOp.TeleOpState.IDLE,driverGamepad.triangle.pressed(), robot.storage.outtakeMode(1))
                .transition(BaseTeleOp.TeleOpState.IDLE, BaseTeleOp.TeleOpState.IDLE,driverGamepad.square.pressed(), robot.storage.nextBall())
                .transition(BaseTeleOp.TeleOpState.IDLE, BaseTeleOp.TeleOpState.IDLE,driverGamepad.circle.pressed(),
                        new InstantCommand(()->robot.turret.setTargetVelocity(velocity))
                )
                .transition(BaseTeleOp.TeleOpState.IDLE, BaseTeleOp.TeleOpState.IDLE,driverGamepad.dpad_up.pressed(),
                        new InstantCommand(()->robot.turret.hood.setTarget(angle))
                );
        FSM<BaseTeleOp.TeleOpState> fsm = fsmBuilder.build(scheduler);
        MovingAverageFilter loopTimeFilter=new MovingAverageFilter(50);

        while (opModeIsActive()) {
            robot.read();

            telemetry.addLine("Pose:");
            telemetry.addData("x", robot.drive.localizer.getPose().position.x);
            telemetry.addData("y", robot.drive.localizer.getPose().position.y);
            telemetry.addData("heading", robot.drive.localizer.getPose().heading.log());
            telemetry.addData("state", fsm.getCurrentState());
            telemetry.addData("hz", loopTimeFilter.update(1/(Performance.loopTimeNano()/1E9)));
            telemetry.addData("velocity", velocity);
            telemetry.addData("current speed", robot.turret.getCurrentVelocity());
            telemetry.addData("angle", angle);
            telemetry.addData("current angle", robot.turret.hood.getTarget());


            telemetry.addData("x", robot.drive.localizer.getPose().position.x.get(0));
            telemetry.addData("y", robot.drive.localizer.getPose().position.y.get(0));
            telemetry.addData("heading (deg)",  Math.toDegrees(robot.drive.localizer.getPose().heading.value().log()));

            telemetry.addData("Distance: ", robot.turret.getDistanceToTarget(new Pose2d(robot.drive.localizer.getPose().position.value().x, robot.drive.localizer.getPose().position.value().y, robot.drive.localizer.getPose().heading.value().log()),  new Pose2d(60,63, Math.toRadians(-45))));



            telemetry.update();

            fsm.update();
            driverGamepad.process();
            operatorGamepad.process();
        }
    }
}


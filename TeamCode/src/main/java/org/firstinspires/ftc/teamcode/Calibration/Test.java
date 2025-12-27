//package org.firstinspires.ftc.teamcode.calibration;
//
//import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;
//import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.smartcluster.oracleftc.commands.Command;
//import com.smartcluster.oracleftc.commands.CommandScheduler;
//import com.smartcluster.oracleftc.commands.ParallelCommand;
//import com.smartcluster.oracleftc.fsm.FSM;
//import com.smartcluster.oracleftc.math.control.PIDController;
//import com.smartcluster.oracleftc.math.control.TrapezoidalMotionProfile;
//import com.smartcluster.oracleftc.math.filters.MovingAverageFilter;
//import com.smartcluster.oracleftc.utils.ProcessedGamepad;
//
//import org.firstinspires.ftc.teamcode.opmode.teleop.BaseTeleOp;
//import org.firstinspires.ftc.teamcode.subsystem.Robot;
//
//import java.util.List;
//
//@Config
//@TeleOp(group="Calibration")
//public class Test extends LinearOpMode {
//    public static double position = 0;
//    public final double MOTOR_TO_TURRET_RATIO = 260.0 / 48;
//    public final double ENCODER_TICKS_PER_ROTATION = 384.5 * MOTOR_TO_TURRET_RATIO;
//    public final double ENCODER_TICKS_PER_DEGREE = ENCODER_TICKS_PER_ROTATION / 360;
//    public static PIDController pid = new PIDController(0, 0, 0);
//    public static TrapezoidalMotionProfile mp = new TrapezoidalMotionProfile(6000, 10000, 12000);
//    private final CommandScheduler scheduler = new CommandScheduler();
//    private static Pose2d currentPose;
//
//    public enum TeleOpState {
//        INIT, Tracking
//    }
//
//    public static double Kv = 1.1;
//    public static double Ka = 0.2;
//    public static double Ks = 0.001;
//    FeedforwardCoefficients coefficients = new FeedforwardCoefficients(Kv, Ka, Ks);
//    BasicFeedforward shootController = new BasicFeedforward(coefficients);
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        ElapsedTime resetTime = new ElapsedTime();
//
//        Robot robot = new Robot(this);
//        Command.run(robot.reset());
//
//        ProcessedGamepad driverGamepad = new ProcessedGamepad(gamepad1),
//                operatorGamepad = new ProcessedGamepad(gamepad2);
//        waitForStart();
//
//        List<LynxModule> lynxModules = hardwareMap.getAll(LynxModule.class);
//        for (LynxModule lynxModule : lynxModules)
//            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        scheduler.schedule(
//                new ParallelCommand(
//                        robot.turret.ppUpdate(robot.drive.localizer),
//                        robot.drive.drive(driverGamepad)
//
//
//                ));
//        FSM.FSMBuilder<BaseTeleOp.TeleOpState> fsmBuilder =  FSM.<BaseTeleOp.TeleOpState>builder()
//                .initial(BaseTeleOp.TeleOpState.INIT)
//                .state(BaseTeleOp.TeleOpState.INIT,
//                        Command.builder()
//                                .update(()->{
//                                    robot.drive.localizer.update();
//                                    robot.turret.ppToAngle(robot.drive.localizer.getPose(),"RED");
//
//
//                                })
//                                .build()
//
//                );
//        FSM<BaseTeleOp.TeleOpState> fsm = fsmBuilder.build(scheduler);
//        MovingAverageFilter loopTimeFilter=new MovingAverageFilter(50);
//
//        while (opModeIsActive()) {
//
//
////            fsm.update();
//            operatorGamepad.process();
//            telemetry.addData("Current position", robot.turret.rotate.getCurrentPosition().get(0));
//            telemetry.addData("Target pose = ", position);
//
//            telemetry.update();
//
//        }
//    }
//}


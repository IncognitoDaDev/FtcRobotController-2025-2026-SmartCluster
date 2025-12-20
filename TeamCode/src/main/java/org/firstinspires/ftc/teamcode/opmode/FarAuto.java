//package org.firstinspires.ftc.teamcode.opmode;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.canvas.Canvas;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.InstantAction;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.Twist2d;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.RobotLog;
//import com.smartcluster.oracleftc.commands.Command;
//import com.smartcluster.oracleftc.commands.InstantCommand;
//import com.smartcluster.oracleftc.commands.ParallelCommand;
//import com.smartcluster.oracleftc.commands.SequentialCommand;
//import com.smartcluster.oracleftc.commands.ThreadedCommandScheduler;
//import com.smartcluster.oracleftc.commands.WaitCommand;
//import com.smartcluster.oracleftc.hardware.OracleOptimize;
//import com.smartcluster.oracleftc.math.filters.MovingAverageFilter;
//import com.smartcluster.oracleftc.utils.Performance;
//
//import org.firstinspires.ftc.teamcode.subsystem.Robot;
//
//import java.util.ArrayList;
//import java.util.List;
//import java.util.concurrent.Callable;
//import java.util.concurrent.ExecutionException;
//import java.util.concurrent.ExecutorService;
//import java.util.concurrent.Executors;
//import java.util.concurrent.Future;
//
//@OracleOptimize
//@Autonomous
//public class FarAuto extends LinearOpMode {
//
//    private final ThreadedCommandScheduler scheduler = new ThreadedCommandScheduler();
//    private static Action commandToAction(Command c)
//    {
//        return new Action() {
//            private boolean initialized=false;
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                if(!initialized) {
//                    c.init();
//                    initialized = true;
//                }
//                c.update();
//                if(c.finished())
//                {
//                    c.end(false);
//                    return false;
//                }else return true;
//            }
//        };
//    }
//
//    private final Pose2d startPose = new Pose2d(-11, -57.5, Math.toRadians(90));
//    private final Pose2d ShootPose = new Pose2d()
//    private final Pose2d stack1= new Pose2d(-57,60, Math.toRadians(315));
//    private final Pose2d stack2 = new Pose2d(-34,-55.5, Math.toRadians(90)).plus(new Twist2d(new com.acmerobotics.roadrunner.Vector2d(-15.3,0),0));
//    private final Pose2d stack3 = new Pose2d(-48,-25.4, Math.toRadians(90)).plus(new Twist2d(new com.acmerobotics.roadrunner.Vector2d(-15.3,0),0));
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        Robot robot = new Robot(this);
//
//        Command.run(robot.reset());
//
//
//
//
//        scheduler.schedule(robot.update());
//        robot.mecanumDrive.localizer.setPose(startPose);
//        MovingAverageFilter loopTimeFilter=new MovingAverageFilter(50);
//        Action autoAction = new SequentialAction(
////            commandToAction(
////                    new SequentialCommand(
////                            new ParallelCommand(
////                                    new InstantCommand(()->robot.turret.hood.setTarget(0.9)),
////                                    new InstantCommand(()->{robot.turret.setShooterSpeed(1000);}),
////                                    new InstantCommand(()->robot.turret.setAngle(0))),
////
////                            new WaitCommand(200),
////                            new InstantCommand(robot.spindex::FlapperUp),
////                            new WaitCommand(250),
////                            robot.spindex.NextSpace(),
////                            new InstantCommand(robot.spindex::FlapperUp),
////                            new WaitCommand(250),
////                            robot.spindex.NextSpace(),
////                            new InstantCommand(robot.spindex::FlapperUp),
////                            new WaitCommand(250),
////                            robot.spindex.NextSpace(),
////                            new InstantCommand(()->robot.turret.hood.setTarget(0.8))
////                    )
////            ),
//
//                    new SequentialAction(
//                    robot.mecanumDrive.actionBuilder(startPose).setTangent(Math.toRadians(180.0))
//                            .splineToLinearHeading(stack1,  Math.toRadians(140))
//                            .build()
//            )
//        );
//
//
//
//                waitForStart();
//
//       robot.mecanumDrive.localizer.setPose(startPose);
//       Thread.sleep(100);
//       List<LynxModule> lynxModules = hardwareMap.getAll(LynxModule.class);
//       for(LynxModule lynxModule: lynxModules)
//           lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//
//       Canvas c = new com.acmerobotics.dashboard.canvas.Canvas();
//       ExecutorService pool = Executors.newFixedThreadPool(2);
//       List<Callable<Void>>  callables = new ArrayList<>();
//       callables.add(()->{robot.mecanumDrive.localizer.update(); return null;});
//       for(LynxModule lynxModule: lynxModules)
//           if(lynxModule.getSerialNumber().isEmbedded())
//           {
//               callables.add(()->{
//                   lynxModule.clearBulkCache();
//                   lynxModule.getBulkData();
//                   return null;
//               });
//           }
//       autoAction.preview(c);
//       boolean b = true;
//       while (b && !Thread.currentThread().isInterrupted()) {
//           for (Future<Void> future : pool.invokeAll(callables)) {
//               try {
//                   future.get();
//               } catch (ExecutionException e) {
//                   RobotLog.logStackTrace(e);
//               }
//           }
//           TelemetryPacket p = new TelemetryPacket();
//           p.fieldOverlay().getOperations().addAll(c.getOperations());
//
//           b = autoAction.run(p);
//           scheduler.update();
//           FtcDashboard.getInstance().sendTelemetryPacket(p);
//           telemetry.addData("hz", loopTimeFilter.update(1/(Performance.loopTimeNano()/1E9)));
//           telemetry.update();
//       }
//       while(opModeIsActive())
//       {
//           for(LynxModule lynxModule: lynxModules)
//               if(lynxModule.getSerialNumber().isEmbedded())
//               {
//                   lynxModule.clearBulkCache();
//                   lynxModule.getBulkData();
//
//               }
//           scheduler.update();
//           telemetry.addData("hz", loopTimeFilter.update(1/(Performance.loopTimeNano()/1E9)));
//           telemetry.update();
//
//       }
//   }
//
//}

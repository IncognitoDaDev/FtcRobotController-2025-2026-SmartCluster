package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.commands.ParallelCommand;
import com.smartcluster.oracleftc.commands.SequentialCommand;
import com.smartcluster.oracleftc.commands.WaitCommand;
import com.smartcluster.oracleftc.fsm.FSM;
import com.smartcluster.oracleftc.math.filters.MovingAverageFilter;
import com.smartcluster.oracleftc.utils.Performance;
import com.smartcluster.oracleftc.utils.ProcessedGamepad;

//import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Storage;


import java.util.List;

@Config
@TeleOp(group = "TeleOp")
public class BaseTeleOp extends LinearOpMode {
    protected Pose2d cornerCoordinates;
    protected Pose2d endPose;
    protected  Pose2d closeShoot;
    protected Pose2d farShoot;

    private final CommandScheduler scheduler = new CommandScheduler();
    public enum TeleOpState{
        INIT,
        IDLE,
        INTAKE,
        SHOOT,FarShooting,CloseShooting,PARKING,
    }
    private TeleOpState CurrentState = TeleOpState.INIT;

    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);

        Robot robot = new Robot(this);
        ProcessedGamepad driverGamepad = new ProcessedGamepad(gamepad1),
                operatorGamepad = new ProcessedGamepad(gamepad2);

        scheduler.schedule(
                new ParallelCommand(
                        robot.drive.drive(driverGamepad),
                        new InstantCommand(robot.drive.localizer::update),
                        robot.update()
                ));

        //Initialize
        FSM.FSMBuilder<TeleOpState> fsmBuilder =  FSM.<TeleOpState>builder()
                .initial(TeleOpState.INIT)
                .transition(TeleOpState.INIT, TeleOpState.IDLE, this::opModeIsActive,
                        new SequentialCommand(
                                robot.storage.flapperDown(),
                                new InstantCommand(() -> robot.storage.storage.OuttakeFacing = -1)

                ))



                // IDLE -------------------------------------------------------------
                .transition(TeleOpState.IDLE,TeleOpState.IDLE,driverGamepad.touchpad.pressed(),
                        new SequentialCommand(
                                new InstantCommand(()->robot.drive.localizer.setPose(new Pose2d(-60,-60,0)))
                        )
                )
                .transition(BaseTeleOp.TeleOpState.IDLE, BaseTeleOp.TeleOpState.INTAKE, driverGamepad.left_bumper.down(),
                        new SequentialCommand(
                                robot.storage.intakeMode(),
                                robot.intake.intake()
                        ))
                .state(BaseTeleOp.TeleOpState.INTAKE, Command.builder()
                        .update(()->{
                            Storage.ArtifactColor frontScan = robot.storage.identifyObjFrontSensor();
                            if (frontScan != Storage.ArtifactColor.EMPTY)
                                robot.storage.storage.appendBallIntake(frontScan);
                        })
                        .build())
                .transition(TeleOpState.INTAKE, TeleOpState.INTAKE, driverGamepad.square::get,
                                robot.storage.nextBall())
                .transition(TeleOpState.INTAKE,TeleOpState.IDLE, driverGamepad.left_bumper.up(),
                        new SequentialCommand(
                            robot.intake.outake(),
                            new WaitCommand(200),
                            robot.intake.stop(),
                            robot.storage.outtakeMode(-1)
                ))
                .transition(TeleOpState.IDLE,TeleOpState.IDLE,driverGamepad.square.pressed(),
                        robot.storage.nextBall())
                .transition(TeleOpState.IDLE,TeleOpState.IDLE,driverGamepad.circle.pressed(),
                        robot.storage.previousBall())

                //Charge init
                .transition(TeleOpState.IDLE,TeleOpState.FarShooting,driverGamepad.dpad_down.pressed(),//👍
                        new ParallelCommand(
                                new InstantCommand(()->robot.turret.setTargetVelocity(4400)),
                                new InstantCommand(()->Actions.runBlocking(robot.drive.actionBuilder(robot.drive.localizer.getPose())
                                        .setTangent(Math.toRadians(90))
                                        .splineToLinearHeading(farShoot,Math.toRadians(55))
                                        .build()))

                        ))


                .transition(TeleOpState.FarShooting,TeleOpState.SHOOT,driverGamepad.cross.pressed(),
                        new SequentialCommand(
                                robot.turret.WaitForRPM(100),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.45)),
                                robot.storage.BallToOuttake(),
                                robot.storage.nextBall(),
                                robot.turret.WaitForRPM(100),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.48)),
                                robot.storage.BallToOuttake(),
                                robot.storage.nextBall(),
                                robot.turret.WaitForRPM(100),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.49)),
                                robot.storage.BallToOuttake()
                                ))
                .transition(TeleOpState.FarShooting,TeleOpState.SHOOT,driverGamepad.square.pressed(),
                        new SequentialCommand(
                                robot.storage.sort(0),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.45)),
                                robot.turret.WaitForRPM(250),
                                robot.storage.BallToOuttake(),
                                robot.storage.sort(1),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.48)),
                                robot.turret.WaitForRPM(250),
                                robot.storage.BallToOuttake(),
                                robot.storage.sort(2),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.49)),
                                robot.turret.WaitForRPM(250),
                                robot.storage.BallToOuttake(),

                                new InstantCommand(() -> robot.turret.setTargetVelocity(0))
                        ))


                .transition(TeleOpState.IDLE,TeleOpState.CloseShooting,driverGamepad.dpad_up.pressed(),
                        new ParallelCommand(
                                new InstantCommand(()->robot.turret.hood.setTarget(0.46)),
                                new InstantCommand(()->{robot.turret.setTargetVelocity(2800);}),
                                new InstantCommand(()->Actions.runBlocking(robot.drive.actionBuilder(robot.drive.localizer.getPose())
                                        .setTangent(Math.toRadians(90))
                                        .splineToLinearHeading(closeShoot,Math.toRadians(90))
                                        .build()))
                                //Experimental auto-positioning
                        )
                )

                .transition(TeleOpState.CloseShooting,TeleOpState.SHOOT,driverGamepad.cross.pressed(),
                        new SequentialCommand(
                                robot.turret.WaitForRPM(100),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.45)),
                                robot.storage.BallToOuttake(),
                                robot.storage.nextBall(),
                                robot.turret.WaitForRPM(100),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.48)),
                                robot.storage.BallToOuttake(),
                                robot.storage.nextBall(),
                                robot.turret.WaitForRPM(100),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.49)),
                                robot.storage.BallToOuttake()
                        ))
                .transition(TeleOpState.CloseShooting,TeleOpState.SHOOT,driverGamepad.square.pressed(),
                        new SequentialCommand(
                                robot.storage.sort(0),
                                robot.turret.WaitForRPM(250),
                                robot.storage.BallToOuttake(),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.48)),
                                robot.storage.sort(1),
                                robot.turret.WaitForRPM(250),
                                robot.storage.BallToOuttake(),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.5)),
                                robot.storage.sort(2),
                                robot.turret.WaitForRPM(250),
                                robot.storage.BallToOuttake()
                        ))

                .transition(TeleOpState.SHOOT, TeleOpState.IDLE, () -> CurrentState == TeleOpState.SHOOT,
                        new SequentialCommand(
                                new InstantCommand(()->{
                                            robot.turret.setTargetVelocity(0);
                                })
                        ));



        FSM<TeleOpState> fsm = fsmBuilder.build(scheduler);

        waitForStart();

        List<LynxModule> lynxModules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule lynxModule : lynxModules)
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);


        MovingAverageFilter loopTimeFilter=new MovingAverageFilter(50);

        while (opModeIsActive()) {

            for (LynxModule lynxModule : lynxModules)
            {
                lynxModule.clearBulkCache();
                lynxModule.getBulkData();
            }
            if (gamepad1.ps) {
                Actions.runBlocking(
                        robot.drive.actionBuilder(robot.drive.localizer.getPose())
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(12, 12,Math.toRadians(-135)),Math.toRadians(55))
                                .build()
                );
            }

            CurrentState = fsm.getCurrentState();

            telemetry.addLine("StorageCache:");
            telemetry.addData("Order [0]", robot.cam.getOrder()[0]);
            telemetry.addData("Order [1]", robot.cam.getOrder()[1]);
            telemetry.addData("Order [2]", robot.cam.getOrder()[2]);
            telemetry.addData("[0]", robot.storage.storage.Slot[0]);
            telemetry.addData("[1]", robot.storage.storage.Slot[1]);
            telemetry.addData("[2]", robot.storage.storage.Slot[2]);
            telemetry.addData("x", robot.drive.localizer.getPose().position.x);
            telemetry.addData("y", robot.drive.localizer.getPose().position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(robot.drive.localizer.getPose().heading.toDouble()));
            telemetry.addData("state", CurrentState);
            telemetry.addData("hz", loopTimeFilter.update(1/(Performance.loopTimeNano()/1E9)));
            telemetry.update();

            fsm.update();
            driverGamepad.process();
            operatorGamepad.process();
        }
    }
}
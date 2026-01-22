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
import com.smartcluster.oracleftc.commands.ConditionalCommand;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.commands.ParallelCommand;
import com.smartcluster.oracleftc.commands.RaceCommand;
import com.smartcluster.oracleftc.commands.SequentialCommand;
import com.smartcluster.oracleftc.commands.WaitCommand;
import com.smartcluster.oracleftc.fsm.FSM;
import com.smartcluster.oracleftc.math.filters.MovingAverageFilter;
import com.smartcluster.oracleftc.utils.Performance;
import com.smartcluster.oracleftc.utils.ProcessedGamepad;

//import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Storage;


import java.util.List;

@Config
@TeleOp(group = "TeleOp")
public class BaseTeleOp extends LinearOpMode {
    protected Pose2d cornerCoordinate;
    protected Pose2d endPose;
    protected  Pose2d closeShoot;
    protected Pose2d farShoot;
    protected boolean isRed=false;
    protected boolean resetEncoder=false;
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

        Robot robot = new Robot(this,isRed);
        ProcessedGamepad driverGamepad = new ProcessedGamepad(gamepad1),
                operatorGamepad = new ProcessedGamepad(gamepad2);
        robot.drive.localizer.setPose(MecanumDrive.currentPose);
        scheduler.schedule(
                new ParallelCommand(
                        robot.drive.driveFieldCentric(driverGamepad, isRed, cornerCoordinate),
                         robot.update()
                ));

        //Initialize
        FSM.FSMBuilder<TeleOpState> fsmBuilder =  FSM.<TeleOpState>builder()
                .initial(TeleOpState.INIT)
                .transition(TeleOpState.INIT, TeleOpState.IDLE, this::opModeIsActive,
                        new SequentialCommand(
                                robot.reset(),
                                new InstantCommand(() -> robot.storage.storage.OuttakeFacing = -1),
                                new InstantCommand(()->robot.turret.turret.setTarget(0)),
                                new InstantCommand(()->robot.turret.setTargetVelocity(500))


                ))
                .transition(BaseTeleOp.TeleOpState.IDLE, BaseTeleOp.TeleOpState.INTAKE, driverGamepad.left_bumper.down(),
                        new SequentialCommand(
                                robot.storage.intakeMode(),
                                robot.intake.intake()
                        ))
                .state(BaseTeleOp.TeleOpState.INTAKE, Command.builder()
                        .update(()->{
                            Storage.ArtifactColor frontScan = robot.storage.identifyObjFrontSensor();
                            robot.storage.storage.appendBallIntake(frontScan);
//                            if (frontScan != Storage.ArtifactColor.EMPTY)
//                                robot.storage.storage.appendBallIntake(frontScan);
                        })
                        .build())
                .transition(TeleOpState.INTAKE, TeleOpState.INTAKE, driverGamepad.square::get,
                                robot.storage.nextBall())
                .transition(TeleOpState.INTAKE,TeleOpState.INTAKE,driverGamepad.circle.pressed(),
                        robot.storage.previousBall())
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
                        new InstantCommand(()->robot.turret.setTargetVelocity(3500))
                        )

                .transition(TeleOpState.FarShooting,TeleOpState.SHOOT,driverGamepad.cross.pressed(),
                        new SequentialCommand(
                                robot.turret.WaitForRPM(500),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.67)),
                                robot.storage.BallToOuttake(),
                                robot.storage.nextBall(),
                                robot.turret.WaitForRPM(500),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.69)),
                                robot.storage.BallToOuttake(),
                                robot.storage.nextBall(),
                                robot.turret.WaitForRPM(500),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.71)),
                                robot.storage.BallToOuttake()

                                )
                        )
                .transition(TeleOpState.FarShooting,TeleOpState.SHOOT,driverGamepad.square.pressed(),
                        new RaceCommand(
                                new SequentialCommand(
                                robot.storage.sort(0),
                                robot.turret.WaitForRPM(2000),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.67)),
                                robot.storage.BallToOuttake(),
                                robot.storage.sort(1),
                                robot.turret.WaitForRPM(1000),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.69)),
                                robot.storage.BallToOuttake(),
                                robot.storage.sort(2),
                                robot.turret.WaitForRPM(1000),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.71)),
                                robot.storage.BallToOuttake(),

                                new InstantCommand(() -> robot.turret.setTargetVelocity(500))

                                ),
                                robot.storage.BroPleaseStopItsEmpty()

                        ))


                .transition(TeleOpState.IDLE,TeleOpState.CloseShooting,driverGamepad.dpad_up.pressed(),
                        new ParallelCommand(
                                new InstantCommand(()->robot.turret.hood.setTarget(0.58)),
                                new InstantCommand(()->{robot.turret.setTargetVelocity(2700);}),
                                new InstantCommand(()->robot.turret.turret.setTarget(0))
//                                new InstantCommand(()->Actions.runBlocking(robot.drive.actionBuilder(robot.drive.localizer.getPose())
//                                        .setTangent(Math.toRadians(90))
//                                        .splineToLinearHeading(closeShoot,Math.toRadians(90))
//                                        .build()))
                                //Experimental auto-positioning
                        )
                )

                .transition(TeleOpState.CloseShooting,TeleOpState.SHOOT,driverGamepad.cross.pressed(),
                        new SequentialCommand(
                                robot.turret.WaitForRPM(500),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.58)),
                                robot.storage.BallToOuttake(),
                                robot.storage.nextBall(),
                                robot.turret.WaitForRPM(500),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.56)),
                                robot.storage.BallToOuttake(),
                                robot.storage.nextBall(),
                                robot.turret.WaitForRPM(500),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.54)),
                                robot.storage.BallToOuttake(),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.55))


                        ))
                .transition(TeleOpState.CloseShooting,TeleOpState.SHOOT,driverGamepad.square.pressed(),
                        new RaceCommand(
                                new SequentialCommand(
                                        robot.storage.sort(0),
                                        robot.turret.WaitForRPM(2000),
                                        new InstantCommand(()->robot.turret.hood.setTarget(0.58)),
                                        robot.storage.BallToOuttake(),
                                        robot.storage.sort(1),
                                        robot.turret.WaitForRPM(1000),
                                        new InstantCommand(()->robot.turret.hood.setTarget(0.56)),
                                        robot.storage.BallToOuttake(),
                                        robot.storage.sort(2),
                                        robot.turret.WaitForRPM(1000),
                                        new InstantCommand(()->robot.turret.hood.setTarget(0.54)),
                                        robot.storage.BallToOuttake(),

                                        new InstantCommand(() -> robot.turret.setTargetVelocity(500))

                                ),
                                robot.storage.BroPleaseStopItsEmpty()

                        ))

                .transition(TeleOpState.SHOOT, TeleOpState.IDLE, () -> CurrentState == TeleOpState.SHOOT,
                        new SequentialCommand(
                                new InstantCommand(()->{
                                            robot.turret.setTargetVelocity(500);
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
            robot.drive.updatePoseEstimate();
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
            telemetry.addData("turret shoot speed", robot.turret.getCurrentVelocity());
            telemetry.addData("heading (deg)", Math.toDegrees(robot.drive.localizer.getPose().heading.toDouble()));
            telemetry.addData("state", CurrentState);
            telemetry.addData("hz", loopTimeFilter.update(1/(Performance.loopTimeNano()/1E9)));
            telemetry.addData("Heading", Math.toDegrees(robot.drive.localizer.getPose().heading.toDouble()));
            telemetry.update();

            fsm.update();
            driverGamepad.process();
            operatorGamepad.process();
        }
    }
}
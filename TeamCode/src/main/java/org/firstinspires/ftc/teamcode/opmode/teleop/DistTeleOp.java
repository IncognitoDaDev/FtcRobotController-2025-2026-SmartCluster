package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.commands.RaceCommand;
import com.smartcluster.oracleftc.commands.SequentialCommand;
import com.smartcluster.oracleftc.commands.WaitCommand;
import com.smartcluster.oracleftc.fsm.FSM;
import com.smartcluster.oracleftc.math.filters.MovingAverageFilter;
import com.smartcluster.oracleftc.utils.Performance;
import com.smartcluster.oracleftc.utils.ProcessedGamepad;

import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Storage;

@Config
@TeleOp(group = "TeleOp")
public class DistTeleOp extends LinearOpMode {
    Pose2d cornerCoordinate = new Pose2d(60,63, Math.toRadians(-45));

    private final CommandScheduler scheduler = new CommandScheduler();
    public enum TeleOpState{
        INIT,
        IDLE,
        INTAKE,
        SHOOT, PRESHOOT,CloseShooting,PARKING,
    }
    private TeleOpState CurrentState = TeleOpState.INIT;


    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);

        Robot robot = new Robot(this, true);
        ProcessedGamepad driverGamepad = new ProcessedGamepad(gamepad1);

        robot.drive.localizer.setPose(MecanumDrive.currentPose);

        scheduler.schedule(
                    robot.update(),
                    robot.drive.driveFieldCentric(driverGamepad, true, cornerCoordinate)
                );

        //Initialize
        FSM<TeleOpState> fsm = FSM.<TeleOpState>builder()
                .initial(TeleOpState.INIT)
                .transition(TeleOpState.INIT, TeleOpState.IDLE, this::opModeIsActive,
                        new SequentialCommand(
                                robot.reset(),
                                new InstantCommand(() -> {
                                    robot.storage.storage.OuttakeFacing = -1;
                                    robot.turret.turret.setTarget(0);
                                    robot.turret.setTargetVelocity(500);
                                    robot.turret.setTracking(robot.drive, cornerCoordinate);
                                })
                ))

                .transition(DistTeleOp.TeleOpState.IDLE, DistTeleOp.TeleOpState.INTAKE, driverGamepad.left_bumper.down(),
                        new SequentialCommand(
                                robot.storage.intakeMode(),
                                robot.intake.intake()
                        ))
                .state(DistTeleOp.TeleOpState.INTAKE, Command.builder()
                        .update(()->{
                                Storage.ArtifactColor frontScan = robot.storage.identifyObj();
                                if (frontScan != Storage.ArtifactColor.EMPTY)
                                {
                                    gamepad1.rumble(35);
                                    robot.storage.storage.appendBallIntake(frontScan);
                                }
                        })
                        .build())
                .transition(TeleOpState.INTAKE, TeleOpState.INTAKE, driverGamepad.square::get,
                                robot.storage.nextBall())
                .transition(TeleOpState.INTAKE, TeleOpState.INTAKE,driverGamepad.circle.pressed(),
                        robot.storage.previousBall())
                .transition(TeleOpState.INTAKE, TeleOpState.IDLE, driverGamepad.left_bumper.up(),
                        new SequentialCommand(
                            robot.intake.outake(),
                            new WaitCommand(100),
                            robot.intake.stop(),
                            robot.storage.outtakeMode(-1)
                ))

                .transition(TeleOpState.IDLE, TeleOpState.IDLE,driverGamepad.square.pressed(),
                        robot.storage.nextBall())
                .transition(TeleOpState.IDLE, TeleOpState.IDLE,driverGamepad.circle.pressed(),
                        robot.storage.previousBall())

                //Charge init
                .transition(TeleOpState.IDLE, TeleOpState.PRESHOOT, driverGamepad.dpad_down.pressed(),//👍
                        new InstantCommand(() -> robot.turret.isAboutToShot.set(true))
                        )

                .transition(TeleOpState.PRESHOOT, TeleOpState.SHOOT, driverGamepad.cross.pressed(),
                        new SequentialCommand(
                                robot.turret.WaitForRPM(2000),
                                robot.storage.BallToOuttake(),

                                robot.storage.nextBall(),
                                robot.turret.WaitForRPM(500),
                                robot.storage.BallToOuttake(),

                                robot.storage.nextBall(),
                                robot.turret.WaitForRPM(500),
                                robot.storage.BallToOuttake()
                        ))

                .transition(TeleOpState.PRESHOOT, TeleOpState.SHOOT, driverGamepad.square.pressed(),
                        new RaceCommand(
                                new SequentialCommand(
                                        robot.storage.sort(0),
                                        robot.turret.WaitForRPM(2000),
                                        robot.storage.BallToOuttake(),

                                        robot.storage.sort(1),
                                        robot.turret.WaitForRPM(500),
                                        robot.storage.BallToOuttake(),

                                        robot.storage.sort(2),
                                        robot.turret.WaitForRPM(500),
                                        robot.storage.BallToOuttake()
                                ),
                                robot.storage.BroPleaseStopItsEmpty()
                        )
                )

                .transition(TeleOpState.SHOOT, TeleOpState.IDLE, () -> CurrentState == TeleOpState.SHOOT,
                        new InstantCommand(() -> robot.turret.isAboutToShot.set(false))
                )
                .build(scheduler);

        waitForStart();

        MovingAverageFilter loopTimeFilter=new MovingAverageFilter(100);

        while (opModeIsActive()) {
            robot.read();

            telemetry.addData("isAboutToShot", robot.turret.isAboutToShot.get());
            telemetry.addData("isInsideTheZone", robot.turret.isInsideTheZone(robot.drive.getPose().value()).get());

            CurrentState = fsm.getCurrentState();
            telemetry.addData("Dex Current", robot.storage.spindexer.getPosition().get(0));
            telemetry.addData("Dex Target", robot.storage.spindexer.getTarget());

            telemetry.addData("Turret Velocity", robot.turret.getCurrentVelocity());

            telemetry.addData("Order [0]", robot.cam.getOrder()[0]);
            telemetry.addData("Order [1]", robot.cam.getOrder()[1]);
            telemetry.addData("Order [2]", robot.cam.getOrder()[2]);
            telemetry.addData("Slot [0]", robot.storage.storage.Slot[0]);
            telemetry.addData("Slot [1]", robot.storage.storage.Slot[1]);
            telemetry.addData("Slot [2]", robot.storage.storage.Slot[2]);

            telemetry.addData("x", robot.drive.localizer.getPose().position.x.get(0));
            telemetry.addData("y", robot.drive.localizer.getPose().position.y.get(0));
            telemetry.addData("heading (deg)",  Math.toDegrees(robot.drive.localizer.getPose().heading.value().log()));
            telemetry.addData("state", CurrentState);

            telemetry.addData("hz", loopTimeFilter.update(1/(Performance.loopTimeNano()/1E9)));
            telemetry.update();

            fsm.update();

            driverGamepad.process();
        }
    }
}
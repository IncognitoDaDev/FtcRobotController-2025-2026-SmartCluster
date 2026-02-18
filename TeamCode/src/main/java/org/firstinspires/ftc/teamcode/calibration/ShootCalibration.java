package org.firstinspires.ftc.teamcode.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.InstantCommand;
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
@TeleOp(group = "Calibration")
public class ShootCalibration extends LinearOpMode {

    private final CommandScheduler scheduler = new CommandScheduler();
    public enum TeleOpState{
        INIT,
        IDLE,
        INTAKE,
        SHOOT
    }
    private TeleOpState CurrentState = TeleOpState.INIT;

    static public double TurrVelocity = 0.0, HoodAngle = 0.0;
    Pose2d RedGoal = new Pose2d(60,63, Math.toRadians(-45));


    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);

        Robot robot = new Robot(this, false);
        ProcessedGamepad driverGamepad = new ProcessedGamepad(gamepad1);

        robot.drive.localizer.setPose(MecanumDrive.currentPose);

        scheduler.schedule(
                    robot.update(),
                    robot.drive.driveFieldCentric(driverGamepad, false, new Pose2d(0, 0, 0))
                );

        //Initialize
        FSM<TeleOpState> fsm = FSM.<TeleOpState>builder()
                .initial(TeleOpState.INIT)
                .transition(TeleOpState.INIT, TeleOpState.IDLE, this::opModeIsActive,
                        new SequentialCommand(
                                robot.reset(),
                                new InstantCommand(() -> {
                                    robot.storage.storage.OuttakeFacing = -1;
                                })
                ))

                .transition(ShootCalibration.TeleOpState.IDLE, ShootCalibration.TeleOpState.INTAKE, driverGamepad.left_bumper.down(),
                        new SequentialCommand(
                                robot.storage.intakeMode(),
                                robot.intake.intake()
                        ))
                .state(ShootCalibration.TeleOpState.INTAKE, Command.builder()
                        .update(()->{
                                Storage.ArtifactColor frontScan = robot.storage.identifyObj();
                                if (frontScan != Storage.ArtifactColor.EMPTY)
                                {
                                    gamepad1.rumble(25);
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

                .transition(TeleOpState.IDLE, TeleOpState.SHOOT,driverGamepad.dpad_down.pressed(),//👍
                        new SequentialCommand(
                                robot.turret.WaitForRPM(1000),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.6)),
                                robot.storage.BallToOuttake(),
                                robot.storage.nextBall()
                        ))

                .transition(TeleOpState.SHOOT, TeleOpState.IDLE, () -> CurrentState == TeleOpState.SHOOT,
                        new SequentialCommand(
                                new InstantCommand(()->{
//                                            robot.turret.setTargetVelocity(500);
                                })
                        ))
                .build(scheduler);

        waitForStart();

        MovingAverageFilter loopTimeFilter=new MovingAverageFilter(100);

        while (opModeIsActive()) {
            robot.read();

            robot.turret.setTargetVelocity(TurrVelocity);
//            robot.turret.setNormalizedHood(HoodAngle);
            robot.turret.hood.setTarget(HoodAngle);
            telemetry.addData("Dist To Target", robot.turret.getDistanceToTarget(robot.drive.getPose().value(), RedGoal));

            CurrentState = fsm.getCurrentState();
//            telemetry.addData("Dex Current", robot.storage.spindexer.getPosition().get(0));
//            telemetry.addData("Dex Target", robot.storage.spindexer.getTarget());

            telemetry.addData("Hood Position", robot.turret.hood.getTarget());
            telemetry.addData("Turret Velocity", robot.turret.getCurrentVelocity());

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
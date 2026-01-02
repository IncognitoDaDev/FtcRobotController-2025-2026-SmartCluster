package org.firstinspires.ftc.teamcode.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.commands.ParallelCommand;
import com.smartcluster.oracleftc.commands.SequentialCommand;
import com.smartcluster.oracleftc.fsm.FSM;
import com.smartcluster.oracleftc.math.filters.MovingAverageFilter;
import com.smartcluster.oracleftc.utils.Performance;
import com.smartcluster.oracleftc.utils.ProcessedGamepad;

import org.firstinspires.ftc.teamcode.opmode.teleop.BaseTeleOp;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Storage;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@Config
@TeleOp(group="Calibration")
public class storageCalibration extends LinearOpMode {
    private final CommandScheduler scheduler = new CommandScheduler();

    @Override
    public void runOpMode() throws InterruptedException {
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

        FSM.FSMBuilder<BaseTeleOp.TeleOpState> fsmBuilder =  FSM.<BaseTeleOp.TeleOpState>builder()
                .initial(BaseTeleOp.TeleOpState.INIT)
                .transition(BaseTeleOp.TeleOpState.INIT, BaseTeleOp.TeleOpState.IDLE, this::opModeIsActive,
                        new SequentialCommand(
                                robot.reset(),
                                new InstantCommand(() -> robot.storage.storage.OuttakeFacing = -1)
                        )
                )
                .state(BaseTeleOp.TeleOpState.INTAKE, Command.builder()
                        .update(()->{
                            Storage.ArtifactColor frontScan = robot.storage.identifyObjFrontSensor();
                            if (frontScan != Storage.ArtifactColor.EMPTY)
                                robot.storage.storage.appendBallIntake(frontScan);
                        })
                        .build())
                .transition(BaseTeleOp.TeleOpState.IDLE, BaseTeleOp.TeleOpState.INTAKE, driverGamepad.left_bumper.down(),
                        new SequentialCommand(
                                robot.storage.intakeMode(),
                                robot.intake.intake()
                        ))
                .transition(BaseTeleOp.TeleOpState.INTAKE, BaseTeleOp.TeleOpState.INTAKE, () -> driverGamepad.square.get(),
                        robot.storage.nextBall())
                .transition(BaseTeleOp.TeleOpState.INTAKE, BaseTeleOp.TeleOpState.IDLE, driverGamepad.left_bumper.up(),
                        new SequentialCommand(
                                robot.intake.stop(),
                                robot.storage.outtakeMode(-1)
                        ))

                .transition(BaseTeleOp.TeleOpState.IDLE, BaseTeleOp.TeleOpState.IDLE, driverGamepad.square.pressed(),
                        robot.storage.nextBall())
                .transition(BaseTeleOp.TeleOpState.IDLE, BaseTeleOp.TeleOpState.IDLE, driverGamepad.circle.pressed(),
                        robot.storage.previousBall())

                .transition(BaseTeleOp.TeleOpState.IDLE, BaseTeleOp.TeleOpState.IDLE, driverGamepad.triangle.pressed(),
                        robot.storage.sort(Storage.ArtifactColor.PURPLE))
                .transition(BaseTeleOp.TeleOpState.IDLE, BaseTeleOp.TeleOpState.IDLE, driverGamepad.cross.pressed(),
                        robot.storage.sort(Storage.ArtifactColor.GREEN));

        FSM<BaseTeleOp.TeleOpState> fsm = fsmBuilder.build(scheduler);

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

            telemetry.addData("StorageState", robot.storage.storage.OuttakeFacing);
            telemetry.addLine("StorageCache:");
            telemetry.addData("[0]", robot.storage.storage.Slot[0]);
            telemetry.addData("[1]", robot.storage.storage.Slot[1]);
            telemetry.addData("[2]", robot.storage.storage.Slot[2]);
            telemetry.addData("state", fsm.getCurrentState());
            telemetry.addData("hz", loopTimeFilter.update(1/(Performance.loopTimeNano()/1E9)));
            telemetry.update();

            fsm.update();
            driverGamepad.process();
            operatorGamepad.process();
        }
    }
}

package org.firstinspires.ftc.teamcode.opmode;

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
import com.smartcluster.oracleftc.hardware.OracleLynxVoltageSensor;
import com.smartcluster.oracleftc.math.Vector2d;
import com.smartcluster.oracleftc.math.filters.MovingAverageFilter;
import com.smartcluster.oracleftc.utils.Performance;
import com.smartcluster.oracleftc.utils.ProcessedGamepad;

//import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Robot;


import java.util.List;
import java.util.concurrent.atomic.AtomicReference;


@Config
@TeleOp(name="DuoMode")
public class DuoMode extends LinearOpMode {
    private final CommandScheduler scheduler = new CommandScheduler();
    public enum TeleOpState{
        INIT,IDLE,CHARGING,SHOOTING

    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);
        Robot robot = new Robot(this);

        ProcessedGamepad driverGamepad = new ProcessedGamepad(gamepad1),
                operatorGamepad = new ProcessedGamepad(gamepad2);
        Command.run(robot.reset());
        while (opModeInInit()) {
            scheduler.update();
            // You can add telemetry here to monitor initialization if needed
            telemetry.addData("Status", "Robot is Initializing and Resetting");
            telemetry.update();
        }

        waitForStart();

        if(!opModeIsActive())return;
        List<LynxModule> lynxModules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule lynxModule : lynxModules)
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        List<OracleLynxVoltageSensor> voltageSensors = hardwareMap.getAll(OracleLynxVoltageSensor.class);
        for (OracleLynxVoltageSensor voltageSensor :
                voltageSensors) {
            voltageSensor.setPolicy(OracleLynxVoltageSensor.OracleLynxVoltageSensorPolicy.CACHED);
            voltageSensor.setVoltageCacheFreshness(100);

        }

        scheduler.schedule(
                new ParallelCommand(
                        //de adaugat restul de comenzi aici
                        robot.drive.drive(driverGamepad),
                        robot.turret.hood.update(),
                        robot.turret.turret.update(),
                        robot.turret.rotation.update()
                ));
        FSM.FSMBuilder<TeleOpState> fsmBuilder =  FSM.<TeleOpState>builder()
                .state(TeleOpState.INIT,
                        new SequentialCommand(
                                robot.turret.turret.move(new AtomicReference<>(0.0)),
                                robot.turret.rotation.move(new AtomicReference<>(0.0))
                        )
                )
                .initial(TeleOpState.INIT)
                .transition(TeleOpState.INIT, TeleOpState.IDLE, this::isStarted, new SequentialCommand(
                        robot.turret.turret.move(new AtomicReference<>(0.0)),
                        robot.turret.rotation.move(new AtomicReference<>(0.0))


                ));
        fsmBuilder = fsmBuilder
                .transition(TeleOpState.IDLE, TeleOpState.CHARGING, operatorGamepad.cross.pressed(),
                        new SequentialCommand(
                                new InstantCommand(()->robot.turret.turret.setTarget(3000.0)),
                                new InstantCommand(()->robot.turret.rotation.setTarget(10.0)),
                                new InstantCommand(()->robot.turret.hood.setTarget(10.0)

                                )));


        FSM<TeleOpState> fsm = fsmBuilder.build(scheduler);
        MovingAverageFilter loopTimeFilter=new MovingAverageFilter(50);



        while (opModeIsActive()) {

            for (LynxModule lynxModule : lynxModules)
                if (lynxModule.getSerialNumber().isEmbedded()) {
                    lynxModule.clearBulkCache();
                    lynxModule.getBulkData();

                }
            telemetry.addData("state", fsm.getCurrentState());
            telemetry.addData("hz", loopTimeFilter.update(1/(Performance.loopTimeNano()/1E9)));
            telemetry.update();

            driverGamepad.process();
            operatorGamepad.process();
            fsm.update();



        }
    }
}

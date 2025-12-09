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
import com.smartcluster.oracleftc.commands.WaitCommand;
import com.smartcluster.oracleftc.fsm.FSM;
import com.smartcluster.oracleftc.hardware.OracleLynxVoltageSensor;
import com.smartcluster.oracleftc.math.Vector2d;
import com.smartcluster.oracleftc.math.filters.MovingAverageFilter;
import com.smartcluster.oracleftc.utils.Performance;
import com.smartcluster.oracleftc.utils.ProcessedGamepad;

//import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Spindexer;


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
        Intake intake = new Intake(this);
        Spindexer spindex = new Spindexer(this);

        ProcessedGamepad driverGamepad = new ProcessedGamepad(gamepad1),
                operatorGamepad = new ProcessedGamepad(gamepad2);

        Command.run(robot.reset());

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
                        robot.drive.drive(driverGamepad)
//                        robot.turret.hood.update(),
////                        robot.turret.turret.update(),
//                        robot.turret.rotation.update()
                ));

        //Initialize
        FSM.FSMBuilder<TeleOpState> fsmBuilder =  FSM.<TeleOpState>builder()
                .initial(TeleOpState.INIT)
                .transition(TeleOpState.INIT, TeleOpState.IDLE, this::opModeIsActive,
                        new SequentialCommand(
//                                robot.turret.turret.move(new AtomicReference<>(0.0)),
//                                robot.turret.rotation.move(new AtomicReference<>(0.0)),
                                new InstantCommand(intake::Reset)

                ));
        //Intake
        fsmBuilder = fsmBuilder
                .transition(TeleOpState.IDLE,TeleOpState.IDLE,driverGamepad.left_bumper.pressed(),
                        new InstantCommand(intake::On)
                        );


        fsmBuilder = fsmBuilder
                .transition(TeleOpState.IDLE,TeleOpState.IDLE,driverGamepad.left_bumper.released(),
                        new InstantCommand(intake::Reset)
                );
        fsmBuilder = fsmBuilder
                .transition(TeleOpState.IDLE,TeleOpState.IDLE,driverGamepad.dpad_left.pressed(),
                        spindex.nextBall(1)
                );
        fsmBuilder = fsmBuilder
                .transition(TeleOpState.IDLE,TeleOpState.IDLE,driverGamepad.dpad_up.released(),
                        spindex.switchPhase()
                );


        //Charge init
        fsmBuilder = fsmBuilder
                .transition(TeleOpState.IDLE, TeleOpState.CHARGING, operatorGamepad.cross.pressed(),//alex e sigma
                        new SequentialCommand(
//                                new InstantCommand(()->robot.turret.setSpeed(3000.0)),
//                                new InstantCommand(()->robot.turret.setRotation(30)),
//                                new InstantCommand(()->robot.turret.hood.setTarget(0.4))

                        ));
        //Charge cancel
        fsmBuilder = fsmBuilder
                .transition(TeleOpState.CHARGING,TeleOpState.IDLE,operatorGamepad.triangle.pressed(),//👍
                            new SequentialCommand(
//                                    new InstantCommand(()->robot.turret.setSpeed(0)),
//                                    new InstantCommand(()->robot.turret.setRotation(0)),
//                                    new InstantCommand(()->robot.turret.hood.setTarget(0.0))

                            )

                );



        FSM<TeleOpState> fsm = fsmBuilder.build(scheduler);
        MovingAverageFilter loopTimeFilter=new MovingAverageFilter(50);



        while (opModeIsActive()) {

            for (LynxModule lynxModule : lynxModules)
                if (lynxModule.getSerialNumber().isEmbedded()) {
                    lynxModule.clearBulkCache();
                    lynxModule.getBulkData();

                }
            driverGamepad.process();
            operatorGamepad.process();
            fsm.update();
            telemetry.addData("state", fsm.getCurrentState());
            telemetry.addData("hz", loopTimeFilter.update(1/(Performance.loopTimeNano()/1E9)));
            telemetry.update();
            scheduler.update();





        }
    }
}

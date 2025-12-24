package org.firstinspires.ftc.teamcode.Calibration;

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
import com.smartcluster.oracleftc.math.filters.MovingAverageFilter;
import com.smartcluster.oracleftc.utils.Performance;
import com.smartcluster.oracleftc.utils.ProcessedGamepad;

import org.firstinspires.ftc.teamcode.subsystem.Robot;

import java.util.List;

@TeleOp(group="Calibration")
public class dexCalibration extends LinearOpMode {
    private final CommandScheduler scheduler = new CommandScheduler();

    private TeleOpState CurrentState = TeleOpState.INIT;

    public enum TeleOpState{
        INIT,IDLE,INTAKE;
    }

    @Override
    public void runOpMode() throws InterruptedException
    {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);
        Robot robot = new Robot(this);
        ProcessedGamepad driverGamepad = new ProcessedGamepad(gamepad1),
                operatorGamepad = new ProcessedGamepad(gamepad2);

        Command.run(
                new SequentialCommand(
                        robot.reset(),
                        robot.spindex.resetRotary()
                )
        );

        waitForStart();
        if(!opModeIsActive()) return;

        //Pre-config Spindex
        robot.spindex.cachedSensor.setting_WALL = false;
//        robot.spindex.cachedSensor.setting_EMPTY = false;

        scheduler.schedule(
                new ParallelCommand(
                        robot.spindex.update(),
                        robot.spindex.flapper.update()
                ));

        List<LynxModule> lynxModules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule lynxModule : lynxModules)
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        List<OracleLynxVoltageSensor> voltageSensors = hardwareMap.getAll(OracleLynxVoltageSensor.class);
        for (OracleLynxVoltageSensor voltageSensor :
                voltageSensors) {
            voltageSensor.setPolicy(OracleLynxVoltageSensor.OracleLynxVoltageSensorPolicy.CACHED);
            voltageSensor.setVoltageCacheFreshness(100);
        }

        FSM.FSMBuilder<TeleOpState> fsmBuilder =  FSM.<TeleOpState>builder()
                .initial(TeleOpState.INIT)
                .transition(TeleOpState.INIT, TeleOpState.IDLE, this::opModeIsActive,
                        new ParallelCommand(
//                                new InstantCommand(robot.spindex::reset),
//                                new InstantCommand(robot.intake::reset)
                        ))

                .transition(TeleOpState.IDLE, TeleOpState.INTAKE, driverGamepad.left_bumper.pressed(),
                        new SequentialCommand(
                                new InstantCommand(()->{
                                    if (robot.spindex.rotaryTargetPos%robot.spindex.ThirdTurn == 0)
                                        robot.spindex.SwitchMode(-1);
                                }),
                                new InstantCommand(robot.intake::intake)
                        )
                )
                .state(TeleOpState.INTAKE, Command.builder()
                        .update(() -> {
                            robot.spindex.cachedSensor.setFront(robot.spindex.IdentifyColor(robot.spindex.rotaryColorSensorF));
                            robot.spindex.cachedSensor.setLeft(robot.spindex.IdentifyColor(robot.spindex.rotaryColorSensorL));
                            robot.spindex.cachedSensor.setRight(robot.spindex.IdentifyColor(robot.spindex.rotaryColorSensorR));
                           })
//                        .finished(() -> robot.spinDex.cachedSensor.getFront() != ColorType.IdentityObject.EMPTY)
                        .build()
                )
                .transition(TeleOpState.INTAKE, TeleOpState.INTAKE, driverGamepad.square.pressed(),
                        new SequentialCommand(
                                robot.spindex.NextSpace(),
                                new InstantCommand(() ->
                                {
                                    robot.spindex.cachedSensor.reset();
                                    gamepad1.rumble(50);
                                })
                        ))
                .transition(TeleOpState.INTAKE, TeleOpState.IDLE,driverGamepad.left_bumper.released(),
                        new InstantCommand(robot.intake::reset))

                //Debugging
                .transition(TeleOpState.IDLE, TeleOpState.IDLE,driverGamepad.triangle.pressed(),
                        new SequentialCommand(
                                new InstantCommand(robot.spindex::sortPurple),
                                new WaitCommand(200),
                                new InstantCommand(() -> robot.spindex.cachedSensor.reset())
                        ))
                .transition(TeleOpState.IDLE, TeleOpState.IDLE,driverGamepad.cross.pressed(),
                        new SequentialCommand(
                                new InstantCommand(robot.spindex::sortGreen),
                                new WaitCommand(200),
                                new InstantCommand(() -> robot.spindex.cachedSensor.reset())
                        ))
                .transition(TeleOpState.IDLE, TeleOpState.IDLE,driverGamepad.right_bumper.pressed(),
                        new SequentialCommand(
                                new InstantCommand(robot.spindex::sortAny),
                                new WaitCommand(200),
                                new InstantCommand(() -> robot.spindex.cachedSensor.reset())
                        ))
                .transition(TeleOpState.IDLE, TeleOpState.IDLE,driverGamepad.dpad_up.pressed(),
                        new InstantCommand(robot.spindex::FlapperUp))
                .transition(TeleOpState.IDLE, TeleOpState.IDLE,driverGamepad.dpad_up.released(),
                        new InstantCommand(robot.spindex::FlapperDown))

                // Move the balls either by 120 or by 60
                .transition(TeleOpState.IDLE, TeleOpState.IDLE,driverGamepad.square.pressed(),
                        robot.spindex.NextSpace())
                .transition(TeleOpState.IDLE, TeleOpState.IDLE,driverGamepad.circle.pressed(),
                        new InstantCommand(()->{robot.spindex.SwitchMode(1);}));

        FSM<TeleOpState> fsm = fsmBuilder.build(scheduler);
        MovingAverageFilter loopTimeFilter=new MovingAverageFilter(50);

        while(opModeIsActive() && !isStopRequested())
        {
            for (LynxModule lynxModule : lynxModules)
                if (lynxModule.getSerialNumber().isEmbedded()) {
                    lynxModule.clearBulkCache();
                    lynxModule.getBulkData();
                }

            CurrentState = fsm.getCurrentState();

            telemetry.addData("Last Obj F_Sensor", robot.spindex.cachedSensor.getFront());
            telemetry.addData("Last Obj L_Sensor", robot.spindex.cachedSensor.getLeft());
            telemetry.addData("Last Obj R_Sensor", robot.spindex.cachedSensor.getRight());

            telemetry.addData("state", CurrentState);
            telemetry.addData("hz", loopTimeFilter.update(1/(Performance.loopTimeNano()/1E9)));
            telemetry.update();

            fsm.update();
            driverGamepad.process();
            operatorGamepad.process();
        }
    }


}

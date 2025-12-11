package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.commands.ParallelCommand;
import com.smartcluster.oracleftc.commands.SequentialCommand;
import com.smartcluster.oracleftc.commands.WaitCommand;
import com.smartcluster.oracleftc.fsm.FSM;
import com.smartcluster.oracleftc.hardware.OracleLynxVoltageSensor;
import com.smartcluster.oracleftc.math.DualNum;
import com.smartcluster.oracleftc.math.filters.MovingAverageFilter;
import com.smartcluster.oracleftc.utils.Performance;
import com.smartcluster.oracleftc.utils.ProcessedGamepad;

//import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.ColorType;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Spindex;


import java.util.List;


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
                        robot.mecanumDrive.drive(driverGamepad),
//                        robot.turret.hood.update(),
                        robot.turret.update()
//                        robot.turret.rotation.update()
                ));



        //Initialize
        FSM.FSMBuilder<TeleOpState> fsmBuilder =  FSM.<TeleOpState>builder()
                .initial(TeleOpState.INIT)
                .transition(TeleOpState.INIT, TeleOpState.IDLE, this::opModeIsActive,
                        new SequentialCommand(
//                                robot.turret.turret.move(new AtomicReference<>(0.0)),
//                                robot.turret.rotation.move(new AtomicReference<>(0.0)),
                                robot.turret.reset(),
                                new InstantCommand(()->robot.turret.rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)),
                                new InstantCommand(robot.intake::reset)
                        ))


                // IDLE -------------------------------------------------------------
                .state(TeleOpState.IDLE,
                        Command.builder()
                                .update(()->{
                            if (robot.spinDex.IdentifyColor(robot.spinDex.rotaryColorSensorF, new ColorType[] {ColorType.Green, ColorType.Purple}))
                            {
                                gamepad2.rumble(200);
                            }})
                            .build()
                        );
        fsmBuilder = fsmBuilder
        .transition(TeleOpState.IDLE,TeleOpState.IDLE,operatorGamepad.left_bumper.pressed(),
                new SequentialCommand(
                        new InstantCommand(robot.intake::Intake)

                ))
        .transition(TeleOpState.IDLE,TeleOpState.IDLE,operatorGamepad.left_bumper.not(),
                new SequentialCommand(
                        new InstantCommand(robot.intake::reset)
                ))
        .transition(TeleOpState.IDLE,TeleOpState.IDLE,operatorGamepad.dpad_left.pressed(),
                new SequentialCommand(
                        new InstantCommand(()->{robot.spinDex.setTarget(robot.spinDex.rotaryTargetPos + robot.spinDex.ThirdTurn);})
                ))
        .transition(TeleOpState.IDLE,TeleOpState.IDLE,operatorGamepad.dpad_right.pressed(),
                new SequentialCommand(
                        new InstantCommand(()->{robot.spinDex.setTarget(robot.spinDex.rotaryTargetPos - robot.spinDex.ThirdTurn);;})
                ))
        .transition(TeleOpState.IDLE,TeleOpState.IDLE,operatorGamepad.dpad_down.pressed(),
                new SequentialCommand(
                        new InstantCommand(()->{robot.spinDex.setTarget(robot.spinDex.rotaryTargetPos + robot.spinDex.ThirdTurn/2);;})
                ))
        .transition(TeleOpState.IDLE,TeleOpState.IDLE,operatorGamepad.dpad_up.pressed(),
                new SequentialCommand(
                        new InstantCommand(()->{ robot.spinDex.setTarget(robot.spinDex.rotaryTargetPos - robot.spinDex.ThirdTurn/2);;})
                ));

        //Charge init
        fsmBuilder = fsmBuilder
                .transition(TeleOpState.IDLE, TeleOpState.CHARGING, operatorGamepad.cross.pressed(),//alex e sigma
                        new SequentialCommand(
                                new InstantCommand(()->robot.turret.setShooter(3000.0)),
                                new InstantCommand(()->robot.turret.setRotation(30)),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.4)),
                                new InstantCommand(robot.spinDex::sortAny)

                        ));
        //Charge cancel
        fsmBuilder = fsmBuilder
                .transition(TeleOpState.CHARGING,TeleOpState.SHOOTING,operatorGamepad.right_bumper.pressed(),//👍
                        new SequentialCommand(
                                new InstantCommand(()->{if(robot.turret.turret.getPosition().equals(3000.0))gamepad2.rumble(200);}),
                                new InstantCommand(robot.spinDex::FlapperUp),
                                new WaitCommand(100),
                                new InstantCommand(robot.spinDex::FlapperDown),
                                new WaitCommand(100),
                                new InstantCommand(() -> {robot.spinDex.setTarget(robot.spinDex.rotaryTargetPos + robot.spinDex.ThirdTurn/2); })
//                                    new InstantCommand(()->robot.turret.setSpeed(0)),
//                                    new InstantCommand(()->robot.turret.setRotation(0)),
//                                    new InstantCommand(()->robot.turret.hood.setTarget(0.0))

                        )


                )
                .transition(TeleOpState.SHOOTING,TeleOpState.CHARGING,operatorGamepad.triangle.pressed(),
                        new SequentialCommand(
                            new InstantCommand(robot.spinDex::sortAny)

                        ));





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
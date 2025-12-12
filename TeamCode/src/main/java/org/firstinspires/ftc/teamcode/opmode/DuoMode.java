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
        INIT,IDLE,INTAKE,CHARGING,SHOOTING
    }
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);
        Robot robot = new Robot(this);
        Intake intake = new Intake(this);


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
                        //robot.spinDex.update(), // Don't execute me here man
                        robot.turret.update()
//                        robot.turret.rotation.update()
                ));



        //Initialize
        FSM.FSMBuilder<TeleOpState> fsmBuilder =  FSM.<TeleOpState>builder()
                .initial(TeleOpState.INIT)
                .transition(TeleOpState.INIT, TeleOpState.IDLE, this::opModeIsActive,
                        new ParallelCommand(
//                                robot.turret.turret.move(new AtomicReference<>(0.0)),
//                                robot.turret.rotation.move(new AtomicReference<>(0.0)),
                                robot.turret.reset(),
                                new InstantCommand(()->robot.turret.rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)),
                                new InstantCommand(robot.spinDex::reset),
                                new InstantCommand(robot.intake::reset)
                        ));


                // IDLE -------------------------------------------------------------
//                .state(TeleOpState.IDLE,
//                        Command.builder()
//                                .update(()->{
//                            if (robot.spinDex.IdentifyColor(robot.spinDex.rotaryColorSensorF, new ColorType[] {ColorType.Green, ColorType.Purple}))
//                            {
//                                gamepad2.rumble(20);
//                            }})
//                            .build()
//                        );
        fsmBuilder = fsmBuilder
            .transition(TeleOpState.IDLE,TeleOpState.INTAKE,driverGamepad.right_trigger.when(v->v>0.5),

                        new InstantCommand(intake::Intake)

                );
        fsmBuilder = fsmBuilder
            .transition(TeleOpState.INTAKE,TeleOpState.IDLE,driverGamepad.right_trigger.when(v->v<0.5),

                        new InstantCommand(intake::reset)
                );
        fsmBuilder = fsmBuilder
            .transition(TeleOpState.IDLE,TeleOpState.IDLE,operatorGamepad.dpad_left.pressed(),

                    new SequentialCommand(
                            new InstantCommand(()->{robot.spinDex.setTarget(robot.spinDex.rotaryTargetPos + robot.spinDex.ThirdTurn);}),
                            new InstantCommand(robot.spinDex::update)
                    )
                );
        fsmBuilder = fsmBuilder
                .transition(TeleOpState.IDLE,TeleOpState.IDLE,operatorGamepad.dpad_right.pressed(),
                        new SequentialCommand(
                                new InstantCommand(()->{
                                    robot.spinDex.setTarget(robot.spinDex.rotaryTargetPos - robot.spinDex.ThirdTurn);

                                }),
                                new InstantCommand(robot.spinDex::update)
                        )
                );
        fsmBuilder = fsmBuilder

                .transition(TeleOpState.IDLE,TeleOpState.IDLE,operatorGamepad.dpad_down.pressed(),
                        new SequentialCommand(
                                new InstantCommand(()->{
                                    robot.spinDex.setTarget(robot.spinDex.rotaryTargetPos + robot.spinDex.ThirdTurn/2);

                                }),
                                new InstantCommand(robot.spinDex::update)
                        ));
//        fsmBuilder = fsmBuilder
//                .transition(TeleOpState.IDLE,TeleOpState.IDLE,operatorGamepad.dpad_up.pressed(),
//                        new InstantCommand(()->{ robot.spinDex.setTarget(robot.spinDex.rotaryTargetPos - robot.spinDex.ThirdTurn/2);;})
//                );

        //Charge init
        fsmBuilder = fsmBuilder
                .transition(TeleOpState.IDLE, TeleOpState.CHARGING, operatorGamepad.cross.pressed(),//alex e sigma
                        new ParallelCommand(
                                new InstantCommand(()->robot.turret.setShooter(3000.0)),
                                new InstantCommand(()->robot.turret.setRotation(30)),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.4)),
                                new InstantCommand(robot.spinDex::sortAny)

                        ));
        //Shoot
        fsmBuilder = fsmBuilder
                .transition(TeleOpState.CHARGING,TeleOpState.SHOOTING,operatorGamepad.right_bumper.pressed(),//👍
                        new SequentialCommand(
                                new InstantCommand(()->{if(robot.turret.getSpeed()==3000.0)gamepad2.rumble(200);}),
                                new InstantCommand(robot.spinDex::FlapperUp),
                                new WaitCommand(100),
                                new InstantCommand(robot.spinDex::FlapperDown),
                                new WaitCommand(100),
                                new InstantCommand(() -> {robot.spinDex.setTarget(robot.spinDex.rotaryTargetPos + robot.spinDex.ThirdTurn/2); }),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.0))

                        )
                );
        //Next Ball
        fsmBuilder = fsmBuilder
                .transition(TeleOpState.SHOOTING,TeleOpState.SHOOTING,robot.spinDex.flapperIsDown(),
                        new SequentialCommand(
                            new InstantCommand(robot.spinDex::sortAny)

                        ));
        fsmBuilder = fsmBuilder
                .transition(TeleOpState.SHOOTING,TeleOpState.IDLE,robot.spinDex.sortEmpty(),
                        new SequentialCommand(
                            new InstantCommand(robot.spinDex::FlapperDown),
                            new ParallelCommand(
                                new InstantCommand(robot.spinDex::FixOrientationForIntake),
                                new InstantCommand(()->{robot.turret.setShooter(0);}),
                                new InstantCommand(()->{robot.turret.setRotation(0);})

                        )));


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
            telemetry.addData("CurrentPosition", robot.spinDex.getPosition());
            telemetry.addData("TargetPosition", robot.spinDex.rotaryTargetPos);
            telemetry.addData("ErrorDistance", Math.abs(robot.spinDex.rotaryTargetPos-robot.spinDex.getPosition()));
            telemetry.update();
            scheduler.update();





        }
    }
}
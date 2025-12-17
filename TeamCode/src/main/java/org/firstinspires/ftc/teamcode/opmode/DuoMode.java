package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.commands.ParallelCommand;
import com.smartcluster.oracleftc.commands.SequentialCommand;
import com.smartcluster.oracleftc.commands.WaitCommand;
import com.smartcluster.oracleftc.fsm.FSM;
import com.smartcluster.oracleftc.hardware.OracleLynxVoltageSensor;
import com.smartcluster.oracleftc.hardware.wrappers.Encoder;
import com.smartcluster.oracleftc.hardware.wrappers.RawEncoder;
import com.smartcluster.oracleftc.math.filters.MovingAverageFilter;
import com.smartcluster.oracleftc.utils.Performance;
import com.smartcluster.oracleftc.utils.ProcessedGamepad;

//import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.NeoSpindexer;
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
        ProcessedGamepad driverGamepad = new ProcessedGamepad(gamepad1),
                operatorGamepad = new ProcessedGamepad(gamepad2);

        //Command.run(robot.reset());

        waitForStart();

        if(!opModeIsActive())return;

        robot.mecanumDrive.localizer.setPose(new Pose2d(-11,-57.5,Math.toRadians(270)));

        List<LynxModule> lynxModules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule lynxModule : lynxModules)
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

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
                        robot.spinDex.update(),
                        robot.turret.update(),
                        robot.spinDex.flapper.update()
//                        robot.turret.rotation.update()
                ));



        //Initialize
        FSM.FSMBuilder<TeleOpState> fsmBuilder =  FSM.<TeleOpState>builder()
                .initial(TeleOpState.INIT)
                .transition(TeleOpState.INIT, TeleOpState.IDLE, this::opModeIsActive,
                        new ParallelCommand(
                                new InstantCommand(()->robot.turret.rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)),
                                new InstantCommand(robot.spinDex::reset),
                                new InstantCommand(robot.intake::reset)
                        ))


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
            .transition(TeleOpState.IDLE,TeleOpState.INTAKE,driverGamepad.left_bumper.pressed(),
                    new InstantCommand(robot.intake::intake)
                )
            .transition(TeleOpState.INTAKE,TeleOpState.IDLE,driverGamepad.left_bumper.released(),
                    new SequentialCommand(
                    new InstantCommand(robot.intake::reset),
                    new InstantCommand(()->{robot.turret.setAngle(0);})
                ))
            .transition(TeleOpState.IDLE,TeleOpState.IDLE,driverGamepad.square.pressed(),
                    new SequentialCommand(
                            new InstantCommand(robot.spinDex::NextSpace)
                    ))
            .transition(TeleOpState.IDLE,TeleOpState.IDLE,driverGamepad.triangle.pressed(),
                    new SequentialCommand(
                            new InstantCommand(()->{robot.spinDex.SwitchMode(1);})
                    ))
            .transition(TeleOpState.IDLE,TeleOpState.IDLE,driverGamepad.cross.pressed(),
                    new SequentialCommand(
                            new InstantCommand(robot.spinDex::FlapperUp)
                    ))
            .transition(TeleOpState.IDLE,TeleOpState.IDLE,driverGamepad.cross.released(),
                    new SequentialCommand(
                            new InstantCommand(robot.spinDex::FlapperDown)
                    ))
            .transition(TeleOpState.IDLE,TeleOpState.IDLE,operatorGamepad.dpad_right.pressed(),
                        new InstantCommand(()->robot.turret.setTargetSpeed(5000))
            )
                .transition(TeleOpState.IDLE, TeleOpState.CHARGING,operatorGamepad.right_bumper.pressed(),
                        new InstantCommand(()->robot.turret.setTargetSpeed(4000))
              )
           /* .transition(TeleOpState.IDLE,TeleOpState.IDLE,operatorGamepad.dpad_left.pressed(),
                    new SequentialCommand(
                            new InstantCommand(robot.spinDex::NextSpace) // 🤫🤤😃a
                    ))
            .transition(TeleOpState.IDLE,TeleOpState.IDLE,operatorGamepad.dpad_right.pressed(),
                    new SequentialCommand(
                            new InstantCommand(robot.spinDex::PrevSpace)
                    ))

            .transition(TeleOpState.IDLE,TeleOpState.IDLE,operatorGamepad.dpad_down.pressed(),
                    new SequentialCommand(
                            new InstantCommand(()->{robot.spinDex.SwitchMode(1);}))
            )*/
//        fsmBuilder = fsmBuilder
//                .transition(TeleOpState.IDLE,TeleOpState.IDLE,operatorGamepad.dpad_up.pressed(),
//                        new InstantCommand(()->{ robot.spinDex.setTarget(robot.spinDex.rotaryTargetPos - robot.spinDex.ThirdTurn/2);;})
//                );

        //Charge init
            .transition(TeleOpState.IDLE, TeleOpState.CHARGING, operatorGamepad.cross.pressed(),//alex e sigma
                        new SequentialCommand(
                                new InstantCommand(()->robot.turret.setTargetSpeed(5250)),
                                new InstantCommand(()->robot.turret.setAngle(0)),
                                new InstantCommand(()->robot.turret.hood.setTarget(10)),
                                new InstantCommand(robot.spinDex::FixOrientationForOuttake)

                        ))
            .state(TeleOpState.CHARGING,
                        Command.builder()
                                .update(()->{
                                    robot.turret.setTargetSpeed(5250);

                                    double angle = operatorGamepad.left_stick.get().y*270;
                                    robot.turret.setAngle(angle);


                                    telemetry.addData("Robot turret position", robot.turret.getRotation() );
                                })
                                .build()
                        )
             .transition(TeleOpState.CHARGING, TeleOpState.CHARGING, operatorGamepad.left_bumper.pressed(),//alex e sigma
                        new InstantCommand(robot.spinDex::NextSpace)

                        )
                .transition(TeleOpState.CHARGING, TeleOpState.CHARGING, operatorGamepad.right_bumper.pressed(),//alex e sigma
                        new InstantCommand(()->{robot.turret.ppToAngle(robot.pinpoint.getPose(),"RED");})

                )
            .transition(TeleOpState.CHARGING,TeleOpState.SHOOTING,operatorGamepad.triangle.pressed(),//👍
                        new SequentialCommand(
                                new InstantCommand(()->{robot.turret.ppToAngle(robot.pinpoint.getPose(), "RED");}),
                                new InstantCommand(robot.spinDex::FlapperUp),
                                new WaitCommand(100),
                                new InstantCommand(robot.spinDex::FlapperDown),
                                new WaitCommand(100),
                                new InstantCommand(robot.spinDex::NextSpace),
                                new WaitCommand(200),
                                new InstantCommand(robot.spinDex::FlapperUp),
                                new WaitCommand(100),
                                new InstantCommand(robot.spinDex::FlapperDown),
                                new WaitCommand(100),
                                new InstantCommand(robot.spinDex::NextSpace),
                                new WaitCommand(200),

                                new InstantCommand(robot.spinDex::FlapperUp),
                                new WaitCommand(100),
                                new InstantCommand(robot.spinDex::FlapperDown),
                                new WaitCommand(100),
                                new InstantCommand(() -> {robot.spinDex.SwitchMode(1); }),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.0))

                        )
                )


                .transition(TeleOpState.SHOOTING,TeleOpState.IDLE,operatorGamepad.circle.pressed(),
                        new SequentialCommand(
                            new InstantCommand(robot.spinDex::FlapperDown),
                            new ParallelCommand(
                                new InstantCommand(()->{robot.turret.setShooterPower(0);})

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

            robot.mecanumDrive.localizer.update();
            fsm.update();
            driverGamepad.process();
            operatorGamepad.process();
        }
    }
}
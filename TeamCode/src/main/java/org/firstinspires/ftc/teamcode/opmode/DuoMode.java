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
import org.firstinspires.ftc.teamcode.subsystem.ColorType;
import org.firstinspires.ftc.teamcode.subsystem.NeoSpindexer;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Spindex;


import java.util.List;


@Config
@TeleOp(name="DuoMode")
public class DuoMode extends LinearOpMode {
    private final CommandScheduler scheduler = new CommandScheduler();
    public enum TeleOpState{
        INIT,IDLE,INTAKE,CHARGING,SHOOTING,TRACKING
    }
    private TeleOpState CurrentState = TeleOpState.INIT;
    private double AimTolerance = 10, oldAngle = -1;
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
//                        robot.turret.update(),
                        robot.turret.ppUpdate(robot.mecanumDrive.localizer),
                        robot.spinDex.flapper.update()
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
            .transition(TeleOpState.IDLE,TeleOpState.INTAKE,driverGamepad.left_bumper.pressed(),
                    new SequentialCommand(
                            new InstantCommand(robot.intake::intake)
                    )
                )
                .state(TeleOpState.INTAKE, Command.builder()
                        .update(() -> {
                            robot.spinDex.cachedSensor.setFront(robot.spinDex.IdentifyColor(robot.spinDex.rotaryColorSensorF));
                        })
                        .finished(() -> robot.spinDex.cachedSensor.getFront() != ColorType.IdentityObject.EMPTY)
                        .build()
                )
                .transition(TeleOpState.INTAKE, TeleOpState.INTAKE, () ->
                                robot.spinDex.cachedSensor.getFront() != ColorType.IdentityObject.EMPTY && robot.spinDex.cachedSensor.getFront() != ColorType.IdentityObject.WALL,
                        new SequentialCommand(
                                robot.spinDex.NextSpace(),
                                new InstantCommand(() ->
                                {
                                    robot.spinDex.cachedSensor.reset();
                                    gamepad1.rumble(50);
                                }),
                                new WaitCommand(200)
                        ))
            .transition(TeleOpState.INTAKE,TeleOpState.IDLE,driverGamepad.left_bumper.released(),
                    new SequentialCommand(
                    new InstantCommand(robot.intake::reset)
                    //new InstantCommand(()->{robot.turret.setAngle(0);})
                ))
            .transition(TeleOpState.IDLE,TeleOpState.IDLE,driverGamepad.square.pressed(),
                    new SequentialCommand(
                            robot.spinDex.NextSpace()
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
                .transition(TeleOpState.IDLE,TeleOpState.TRACKING,operatorGamepad.dpad_left.pressed(),
                    new SequentialCommand(
                        new InstantCommand(()->{robot.mecanumDrive.localizer.update();}),
                        new InstantCommand(()->{robot.turret.ppToAngle(robot.mecanumDrive.localizer.getPose(),"RED");})
                ))
                .transition(TeleOpState.TRACKING,TeleOpState.IDLE,operatorGamepad.dpad_left.released(),
                    new InstantCommand(()->{robot.turret.setAngle(0);})
                        )
                .transition(TeleOpState.IDLE, TeleOpState.CHARGING,operatorGamepad.right_bumper.pressed(),
                        new InstantCommand(()->robot.turret.setTargetSpeed(4000))
              )

        //Charge init
            .transition(TeleOpState.IDLE, TeleOpState.CHARGING, operatorGamepad.cross.pressed(),//alex e sigma
                        new SequentialCommand(
                                new InstantCommand(()->robot.turret.setTargetSpeed(5600)),
                                new InstantCommand(()->robot.turret.hood.setTarget(10)),
//                                new InstantCommand(()-> robot.spinDex.SwitchMode(-1)),
                                new InstantCommand(()->{robot.mecanumDrive.localizer.update();}),
                                new InstantCommand(()->{robot.turret.ppToAngle(robot.mecanumDrive.localizer.getPose(),"RED");})
                                ))
//            .state(TeleOpState.CHARGING,
//                        Command.builder()
//                                .init(() ->
//                                {
//                                    robot.turret.setTargetSpeed(5250);
//                                    oldAngle = robot.turret.targetAngle;
//                                })
//                                .update(()->{
//                                    double angle = operatorGamepad.left_stick.get().y*270;
//
//                                    if (Math.abs(angle-oldAngle) > AimTolerance)
//                                    {
//                                        robot.turret.setAngle(angle);
//                                        oldAngle = angle;
//                                    }
//
//                                    telemetry.addData("Robot turret position", robot.turret.getRotation() );
//                                })
//                                .build()
//                        )
             .transition(TeleOpState.CHARGING, TeleOpState.CHARGING, operatorGamepad.left_bumper.pressed(),//alex e sigma
                        new InstantCommand(robot.spinDex::NextSpace)

                        )
                .transition(TeleOpState.CHARGING, TeleOpState.CHARGING, operatorGamepad.right_bumper.pressed(),//alex e sigma
                        new InstantCommand(()->{robot.turret.ppToAngle(robot.mecanumDrive.localizer.getPose(),"RED");})

                )
            .transition(TeleOpState.CHARGING,TeleOpState.SHOOTING,operatorGamepad.triangle.pressed(),//👍
                        new SequentialCommand(
                                new InstantCommand(()->{robot.mecanumDrive.localizer.update();}),
                                new InstantCommand(()->{
                                    robot.turret.ppToAngle(robot.mecanumDrive.localizer.getPose(), "RED");

                                    if (robot.spinDex.rotaryTargetPos%robot.spinDex.ThirdTurn != 0)
                                        robot.spinDex.SwitchMode(-1);

                                }),
                                new InstantCommand(robot.spinDex::FlapperUp),
                                new WaitCommand(200),
                                robot.spinDex.NextSpace(),
                                new WaitCommand(200),
                                new InstantCommand(robot.spinDex::FlapperUp),
                                new WaitCommand(200),
                                robot.spinDex.NextSpace(),
                                new WaitCommand(200),
                                new InstantCommand(robot.spinDex::FlapperUp),
                                new WaitCommand(200),
                                robot.spinDex.NextSpace(),
                                new InstantCommand(() -> {robot.spinDex.SwitchMode(1); }),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.0))
                        )
                )
                .transition(TeleOpState.SHOOTING, TeleOpState.IDLE, () -> CurrentState == TeleOpState.SHOOTING,
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

            CurrentState = fsm.getCurrentState();
            telemetry.addData("state", CurrentState);
            telemetry.addData("hz", loopTimeFilter.update(1/(Performance.loopTimeNano()/1E9)));
            telemetry.update();

            robot.mecanumDrive.localizer.update();
            fsm.update();
            driverGamepad.process();
            operatorGamepad.process();
        }
    }
}
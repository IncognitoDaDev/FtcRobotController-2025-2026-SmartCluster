package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
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
import com.smartcluster.oracleftc.math.filters.MovingAverageFilter;
import com.smartcluster.oracleftc.utils.Performance;
import com.smartcluster.oracleftc.utils.ProcessedGamepad;

//import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.subsystem.ColorType;
import org.firstinspires.ftc.teamcode.subsystem.Robot;


import java.util.List;
import java.util.function.Supplier;


@Config
@TeleOp(name="TrackTurret")
public class TrackTurret extends LinearOpMode {
    private final CommandScheduler scheduler = new CommandScheduler();
    public enum TeleOpState{
        INIT,IDLE,INTAKE,SHOOT,FarShooting,CloseShooting,PARKING,
    }
    private TeleOpState CurrentState = TeleOpState.INIT;


    private Supplier<Boolean> isBall(ColorType.IdentityObject Obj)
    {
        return ()-> Obj == ColorType.IdentityObject.GREEN || Obj == ColorType.IdentityObject.PURPLE;
    }

    @Override
    public void runOpMode() throws InterruptedException
    {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);
        Robot robot = new Robot(this);
        ProcessedGamepad driverGamepad = new ProcessedGamepad(gamepad1),
                operatorGamepad = new ProcessedGamepad(gamepad2);

        Command.run(robot.reset());

        waitForStart();

        if(!opModeIsActive())return;

        robot.drive.localizer.setPose(new Pose2d(-60,12, Math.toRadians(0)));

        List<LynxModule> lynxModules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule lynxModule : lynxModules)
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        List<OracleLynxVoltageSensor> voltageSensors = hardwareMap.getAll(OracleLynxVoltageSensor.class);
        for (OracleLynxVoltageSensor voltageSensor :
                voltageSensors) {
            voltageSensor.setPolicy(OracleLynxVoltageSensor.OracleLynxVoltageSensorPolicy.CACHED);
            voltageSensor.setVoltageCacheFreshness(100);
        }

        //de adaugat restul de comenzi aici
        // Adauga comenzi care nu au nevoie de referinte din scriptul asta in Robot.java - R^2-M
        scheduler.schedule(
                new ParallelCommand(
                        robot.drive.drive(driverGamepad),
                        new InstantCommand(robot.drive.localizer::update),
                        robot.turret.ppUpdate(robot.drive.localizer),
//                        robot.turret.update(),
                        robot.spindex.update(),
                        robot.spindex.flapper.update(),
                        robot.turret.hood.update()
                ));



        //Initialize
        FSM.FSMBuilder<TeleOpState> fsmBuilder =  FSM.<TeleOpState>builder()
                .initial(TeleOpState.INIT)
                .transition(TeleOpState.INIT, TeleOpState.IDLE, this::opModeIsActive,
                        new ParallelCommand(
                                new InstantCommand(()->robot.turret.rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)),
                                new InstantCommand(robot.spindex::reset),
                                new InstantCommand(robot.intake::reset)
                        ))

                // IDLE -------------------------------------------------------------
                .transition(TeleOpState.IDLE,TeleOpState.INTAKE,driverGamepad.left_bumper.pressed(),
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
//                            robot.spinDex.cachedSensor.setLeft(robot.spinDex.IdentifyColor(robot.spinDex.rotaryColorSensorL));
//                            robot.spinDex.cachedSensor.setRight(robot.spinDex.IdentifyColor(robot.spinDex.rotaryColorSensorR));
//
//                            telemetry.addData("dexFrontSensorObj", robot.spinDex.cachedSensor.getFront());
//                            telemetry.addData("dexLeftSensorObj", robot.spinDex.cachedSensor.getLeft());
//                            telemetry.addData("dexRightSensorObj", robot.spinDex.cachedSensor.getRight());
                                })
//                        .finished(() -> robot.spinDex.cachedSensor.getFront() != ColorType.IdentityObject.EMPTY)
                                .build()
                )
                .transition(TeleOpState.INTAKE, TeleOpState.INTAKE, () -> robot.spindex.cachedSensor.getFront() == ColorType.IdentityObject.GREEN ||
                                robot.spindex.cachedSensor.getFront() == ColorType.IdentityObject.PURPLE,
                        new SequentialCommand(
                                robot.spindex.NextSpace(),
                                new InstantCommand(() ->
                                {
                                    robot.spindex.cachedSensor.reset();
                                    gamepad1.rumble(50);
                                }),
                                new WaitCommand(100)
                        ))
//                .transition(TeleOpState.INTAKE, TeleOpState.INTAKE, () ->
//                                robot.spinDex.isBall(robot.spinDex.cachedSensor.getFront()).get() &&
//                                        !robot.spinDex.isBall(robot.spinDex.cachedSensor.getRight()).get(),
//                        new SequentialCommand(
//                                robot.spinDex.PreviousSpace(),
//                                new InstantCommand(() ->
//                                {
//                                    robot.spinDex.cachedSensor.reset();
//                                    gamepad1.rumble(50);
//                                }),
//                                new WaitCommand(100)
//                        ))

                .transition(TeleOpState.INTAKE,TeleOpState.IDLE,driverGamepad.left_bumper.released(), new InstantCommand(robot.intake::reset))
                .transition(TeleOpState.IDLE,TeleOpState.IDLE,driverGamepad.square.pressed(), robot.spindex.NextSpace())
                .transition(TeleOpState.IDLE,TeleOpState.IDLE,driverGamepad.circle.pressed(), new InstantCommand(()->{robot.spindex.SwitchMode(1);}))
                //Charge init
                .transition(TeleOpState.IDLE, TeleOpState.FarShooting, driverGamepad.dpad_down.pressed(),
                        new InstantCommand(()->{
//                                    robot.mecanumDrive.localizer.update();
//                                    robot.turret.ppToAngle(robot.mecanumDrive.localizer.getPose(),"RED");
                            if (robot.spindex.rotaryTargetPos%robot.spindex.ThirdTurn != 0)
                                robot.spindex.SwitchMode(-1);

                        })
                )
                .transition(TeleOpState.IDLE,TeleOpState.CloseShooting,driverGamepad.dpad_up.pressed(),
                        new SequentialCommand(
                                new InstantCommand(()->{
//                                    robot.mecanumDrive.localizer.update();
//                                    robot.turret.ppToAngle(robot.mecanumDrive.localizer.getPose(),"RED");
                                    if (robot.spindex.rotaryTargetPos%robot.spindex.ThirdTurn != 0)
                                        robot.spindex.SwitchMode(-1);

                                })
                        ))
                .transition(TeleOpState.FarShooting,TeleOpState.IDLE, driverGamepad.dpad_left.pressed(),
                        new SequentialCommand(
                                new InstantCommand(()->{robot.spindex.SwitchMode(-1);})
                        ))
                .transition(TeleOpState.CloseShooting,TeleOpState.IDLE, driverGamepad.dpad_left.pressed(),
                        new SequentialCommand(
                                new InstantCommand(()->{robot.spindex.SwitchMode(-1);})
                        ))

                .transition(TeleOpState.FarShooting,TeleOpState.FarShooting,driverGamepad.dpad_down.pressed(),//👍
                        new SequentialCommand(
                                new ParallelCommand(
                                        new InstantCommand(()->robot.turret.hood.setTarget(0.7)),
                                        new InstantCommand(()->{robot.turret.setShooterSpeed(5400);
//                                            new InstantCommand(()->robot.turret.setAngle(-60));
                                        }))

                        )
                )
                .transition(TeleOpState.FarShooting,TeleOpState.SHOOT,driverGamepad.x.pressed(),
                        new SequentialCommand(new InstantCommand(robot.spindex::FlapperUp),
                                new WaitCommand(250),
                                robot.spindex.NextSpace(),
                                new InstantCommand(robot.spindex::FlapperUp),
                                new WaitCommand(250),
                                robot.spindex.NextSpace(),
                                new InstantCommand(robot.spindex::FlapperUp),
                                new WaitCommand(250),
                                robot.spindex.NextSpace(),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.8))))
                .transition(TeleOpState.CloseShooting,TeleOpState.CloseShooting,driverGamepad.dpad_up.pressed(),//👍
                        new ParallelCommand(
                                new InstantCommand(()->robot.turret.hood.setTarget(0.6)),
                                new InstantCommand(()->{robot.turret.setShooterSpeed(3000);
//                                    new InstantCommand(()->robot.turret.setAngle(0));
                                })
                        ))




                .transition(TeleOpState.CloseShooting,TeleOpState.SHOOT,driverGamepad.x.pressed(),
                        new SequentialCommand(new InstantCommand(robot.spindex::FlapperUp),
                                new WaitCommand(250),
                                robot.spindex.NextSpace(),
                                new InstantCommand(robot.spindex::FlapperUp),
                                new WaitCommand(250),
                                robot.spindex.NextSpace(),
                                new InstantCommand(robot.spindex::FlapperUp),
                                new WaitCommand(250),
                                robot.spindex.NextSpace(),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.8)))
                )
                .transition(TeleOpState.SHOOT, TeleOpState.IDLE, () -> CurrentState == TeleOpState.SHOOT,
                        new SequentialCommand(
                                new InstantCommand(robot.spindex::FlapperDown),
                                new ParallelCommand(
                                        new InstantCommand(()->{robot.turret.setShooterSpeed(0);}),
                                        new InstantCommand(()->robot.spindex.SwitchMode(-1))
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


            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), robot.drive.localizer.getPose());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            fsm.update();
            driverGamepad.process();
            operatorGamepad.process();
        }
    }
}
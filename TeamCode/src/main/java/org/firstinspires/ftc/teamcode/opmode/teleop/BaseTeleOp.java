package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Spindex_OLD;


import java.util.List;


public class BaseTeleOp extends LinearOpMode {
    protected Pose2d cornerCoordinates;
    private final CommandScheduler scheduler = new CommandScheduler();
    public enum TeleOpState{
        INIT,IDLE,INTAKE,SHOOT,FarShooting,CloseShooting,PARKING,
    }
    private TeleOpState CurrentState = TeleOpState.INIT;

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
        
        if(!opModeIsActive()) return;

        robot.drive.localizer.setPose(new Pose2d(-11,-57.5,Math.toRadians(270)));

        List<LynxModule> lynxModules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule lynxModule : lynxModules)
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        List<OracleLynxVoltageSensor> voltageSensors = hardwareMap.getAll(OracleLynxVoltageSensor.class);
        for (OracleLynxVoltageSensor voltageSensor :
                voltageSensors) {
            voltageSensor.setPolicy(OracleLynxVoltageSensor.OracleLynxVoltageSensorPolicy.CACHED);
            voltageSensor.setVoltageCacheFreshness(100);
        }

        // Adauga comenzi care nu au nevoie de referinte din scriptul asta in Robot.java - R^2-M
        scheduler.schedule(
                new ParallelCommand(
                        robot.drive.drive(driverGamepad),
                        new InstantCommand(robot.drive.localizer::update),
//                        robot.turret.ppUpdate(robot.mecanumDrive.localizer),

                        robot.update()
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
                                if (robot.spindex.rotaryTargetPos% Spindex_OLD.ThirdTurn == 0)
                                    robot.spindex.SwitchMode(-1);
                            }),
                            new InstantCommand(robot.intake::intake)
                    )
                )
                .transition(TeleOpState.INTAKE, TeleOpState.INTAKE, driverGamepad.square.pressed(),
                        new SequentialCommand(
                                robot.spindex.NextSpace(),
                                new InstantCommand(() ->
                                {
                                    //robot.spindex.cachedSensor.reset();
                                    gamepad1.rumble(50);
                                })
                        ))

                .transition(TeleOpState.INTAKE,TeleOpState.IDLE,driverGamepad.left_bumper.released(),robot.intake.stop())
                .transition(TeleOpState.IDLE,TeleOpState.IDLE,driverGamepad.square.pressed(),
                        robot.spindex.NextSpace())
                .transition(TeleOpState.IDLE,TeleOpState.IDLE,driverGamepad.circle.pressed(),
                        new InstantCommand(()->{robot.spindex.SwitchMode(1);}))

                //Charge init
                .transition(TeleOpState.IDLE, TeleOpState.FarShooting, driverGamepad.dpad_down.pressed(),
                                new InstantCommand(()->{
                                    if (robot.spindex.rotaryTargetPos% Spindex_OLD.ThirdTurn != 0)
                                        robot.spindex.SwitchMode(-1);

                                })
                        )
                .transition(TeleOpState.IDLE,TeleOpState.CloseShooting,driverGamepad.dpad_up.pressed(),
                        new SequentialCommand(
                                new InstantCommand(()->{
                                    if (robot.spindex.rotaryTargetPos% Spindex_OLD.ThirdTurn != 0)
                                        robot.spindex.SwitchMode(-1);

                                })
                ))

//                Pentru ce este asta, nici nu vezi ce minge ai in competitie oricum :sob:??
//                .transition(TeleOpState.FarShooting,TeleOpState.IDLE, driverGamepad.dpad_left.pressed(),
//                        new SequentialCommand(
//                                new InstantCommand(()-> robot.spindex.SwitchMode(-1))
//                        ))
//                .transition(TeleOpState.CloseShooting,TeleOpState.IDLE, driverGamepad.dpad_left.pressed(),
//                        new SequentialCommand(
//                                new InstantCommand(()-> robot.spindex.SwitchMode(-1))
//                        ))

                .transition(TeleOpState.FarShooting,TeleOpState.FarShooting,driverGamepad.dpad_down.pressed(),//👍
                        new SequentialCommand(
                                new ParallelCommand(
                                        new InstantCommand(()->robot.turret.setShooterSpeed(5000)),
                                        new InstantCommand(()->robot.turret.setAngle(0))
                                        ))
                        )

                .transition(TeleOpState.FarShooting,TeleOpState.SHOOT,driverGamepad.cross.pressed(),
                        new SequentialCommand(
                                new InstantCommand(()->robot.turret.hood.setTarget(0.65)),
                                new InstantCommand(robot.spindex::FlapperUp),
                                new WaitCommand(300),
                                robot.spindex.NextSpace(),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.67)),
                                new InstantCommand(robot.spindex::FlapperUp),
                                new WaitCommand(300),
                                robot.spindex.NextSpace(),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.71)),
                                new InstantCommand(robot.spindex::FlapperUp),
                                new WaitCommand(300),
                                robot.spindex.NextSpace()))

                .transition(TeleOpState.CloseShooting,TeleOpState.CloseShooting,driverGamepad.dpad_up.pressed(),//👍
                                new ParallelCommand(
                                        new InstantCommand(()->robot.turret.hood.setTarget(0.9)),
                                        new InstantCommand(()->{robot.turret.setShooterSpeed(1000);
                                            new InstantCommand(()->robot.turret.setAngle(0));
                                        })
                                ))
                .transition(TeleOpState.CloseShooting,TeleOpState.SHOOT,driverGamepad.x.pressed(),
                        new SequentialCommand(
                                new InstantCommand(()->robot.turret.hood.setTarget(0.85)),
                                new InstantCommand(robot.spindex::FlapperUp),
                                new WaitCommand(250),
                                robot.spindex.NextSpace(),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.9)),
                                new InstantCommand(robot.spindex::FlapperUp),
                                new WaitCommand(250),
                                robot.spindex.NextSpace(),
                                new InstantCommand(()->robot.turret.hood.setTarget(0.95)),
                                new InstantCommand(robot.spindex::FlapperUp),
                                new WaitCommand(250),
                                robot.spindex.NextSpace()
                ))

                .transition(TeleOpState.SHOOT, TeleOpState.IDLE, () -> CurrentState == TeleOpState.SHOOT,
                        new SequentialCommand(
                            new InstantCommand(robot.spindex::FlapperDown),
                            new ParallelCommand(
                                    new InstantCommand(()->{
                                        robot.turret.setShooterSpeed(0);
                                        robot.spindex.SwitchMode(-1);
                                    })
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

            fsm.update();
            driverGamepad.process();
            operatorGamepad.process();
        }
    }
}
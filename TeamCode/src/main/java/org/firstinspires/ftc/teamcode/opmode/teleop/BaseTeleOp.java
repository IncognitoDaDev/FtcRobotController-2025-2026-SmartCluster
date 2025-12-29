package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
import org.firstinspires.ftc.teamcode.subsystem.CamRecon;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Storage;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;


import org.firstinspires.ftc.teamcode.subsystem.CamRecon;


import java.util.List;


public class  BaseTeleOp extends LinearOpMode {

    protected Pose2d cornerCoordinates;
    private final CommandScheduler scheduler = new CommandScheduler();
    public enum TeleOpState{
        INIT,
        IDLE,
        INTAKE,
        SHOOT,FarShooting,CloseShooting,PARKING,
    }
    private TeleOpState CurrentState = TeleOpState.INIT;

    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);

        Robot robot = new Robot(this);
        CamRecon CamRecon = new CamRecon(this);


        ProcessedGamepad driverGamepad = new ProcessedGamepad(gamepad1),
                operatorGamepad = new ProcessedGamepad(gamepad2);



        scheduler.schedule(
                new ParallelCommand(
                        robot.drive.drive(driverGamepad),
                        new InstantCommand(robot.drive.localizer::update),

                        robot.update()
                ));

        //Initialize
        FSM.FSMBuilder<TeleOpState> fsmBuilder =  FSM.<TeleOpState>builder()
                .initial(TeleOpState.INIT)
                .transition(TeleOpState.INIT, TeleOpState.IDLE, this::opModeIsActive,
                        robot.reset());

//
//                // IDLE -------------------------------------------------------------
//                .transition(TeleOpState.IDLE,TeleOpState.INTAKE,driverGamepad.left_bumper.down(),
//                        new SequentialCommand(
//                                robot.storage.emptyFront(),
//                                robot.intake.intake()
//                        )
//                )
//                .transition(TeleOpState.INTAKE, TeleOpState.INTAKE, ()->driverGamepad.square.get()||robot.storage.front()!=Storage.ArtifactColor.EMPTY,
//                        new SequentialCommand(
//                                robot.storage.nextSpace()
//                        ))
//                .transition(TeleOpState.INTAKE,TeleOpState.IDLE,driverGamepad.left_bumper.released(),robot.intake.stop())
//                .transition(TeleOpState.IDLE,TeleOpState.IDLE,driverGamepad.square.pressed(),
//                        robot.storage.nextSpace())
//                .transition(TeleOpState.IDLE,TeleOpState.IDLE,driverGamepad.circle.pressed(),
//                        robot.storage.previousSpace())
//
//                //Charge init
//                .transition(TeleOpState.IDLE, TeleOpState.FarShooting, driverGamepad.dpad_down.pressed(),
//                        new InstantCommand(()->{
//                            if (robot.storage.rotaryTargetPos% Spindex_OLD.ThirdTurn != 0)
//                                robot.storage.SwitchMode(-1);
//
//                        })
//                )
//                .transition(TeleOpState.IDLE,TeleOpState.CloseShooting,driverGamepad.dpad_up.pressed(),
//                        new SequentialCommand(
//                                new InstantCommand(()->{
//                                    if (robot.storage.rotaryTargetPos% Spindex_OLD.ThirdTurn != 0)
//                                        robot.storage.SwitchMode(-1);
//
//                                })
//                        ))
//
//                .transition(TeleOpState.FarShooting,TeleOpState.FarShooting,driverGamepad.dpad_down.pressed(),//👍
//                        new SequentialCommand(
//                                new ParallelCommand(
//                                        new InstantCommand(()->robot.turret.setShooterSpeed(5000)),
//                                        new InstantCommand(()->robot.turret.setAngle(0))
//                                ))
//                )
//
//                .transition(TeleOpState.FarShooting,TeleOpState.SHOOT,driverGamepad.cross.pressed(),
//                        new SequentialCommand()
//
//                .transition(TeleOpState.CloseShooting,TeleOpState.CloseShooting,driverGamepad.dpad_up.pressed(),
//                        new ParallelCommand(
//                                new InstantCommand(()->robot.turret.hood.setTarget(0.9)),
//                                new InstantCommand(()->{robot.turret.setShooterSpeed(1000);
//                                    new InstantCommand(()->robot.turret.setAngle(0));
//                                })
//                        ))
//                .transition(TeleOpState.CloseShooting,TeleOpState.SHOOT,driverGamepad.x.pressed(),
//                        new SequentialCommand(
//                                new InstantCommand(()->robot.turret.hood.setTarget(0.85)),
//                                new InstantCommand(robot.storage::FlapperUp),
//                                new WaitCommand(250),
//                                robot.storage.NextSpace(),
//                                new InstantCommand(()->robot.turret.hood.setTarget(0.9)),
//                                new InstantCommand(robot.storage::FlapperUp),
//                                new WaitCommand(250),
//                                robot.storage.NextSpace(),
//                                new InstantCommand(()->robot.turret.hood.setTarget(0.95)),
//                                new InstantCommand(robot.storage::FlapperUp),
//                                new WaitCommand(250),
//                                robot.storage.NextSpace()
//                        ))
//
//                .transition(TeleOpState.SHOOT, TeleOpState.IDLE, () -> CurrentState == TeleOpState.SHOOT,
//                        new SequentialCommand(
//                                new InstantCommand(robot.storage::FlapperDown),
//                                new ParallelCommand(
//                                        new InstantCommand(()->{
//                                            robot.turret.setShooterSpeed(0);
//                                            robot.storage.SwitchMode(-1);
//                                        })
//                                )));



        FSM<TeleOpState> fsm = fsmBuilder.build(scheduler);

        waitForStart();


        List<LynxModule> lynxModules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule lynxModule : lynxModules)
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);


        robot.drive.localizer.setPose(new Pose2d(-6,-64,90));


        MovingAverageFilter loopTimeFilter=new MovingAverageFilter(50);

        while (opModeIsActive()) {

            for (LynxModule lynxModule : lynxModules)
            {
                lynxModule.clearBulkCache();
                lynxModule.getBulkData();
            }

            CurrentState = fsm.getCurrentState();

            telemetry.addData("state", CurrentState);
            telemetry.addData("hz", loopTimeFilter.update(1/(Performance.loopTimeNano()/1E9)));
            telemetry.addData(
                    "monolith",
                    java.util.Arrays.toString(CamRecon.getAprilTagPattern())
            );
            telemetry.addData("loc", robot.drive.localizer.getPose());

            telemetry.update();

            fsm.update();
            driverGamepad.process();
            operatorGamepad.process();
        }
    }
}
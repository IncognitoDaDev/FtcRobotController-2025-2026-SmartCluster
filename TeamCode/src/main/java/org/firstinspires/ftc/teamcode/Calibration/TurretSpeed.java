package org.firstinspires.ftc.teamcode.Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.commands.ParallelCommand;
import com.smartcluster.oracleftc.fsm.FSM;
import com.smartcluster.oracleftc.utils.ProcessedGamepad;

import org.firstinspires.ftc.teamcode.opmode.DuoMode;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

import java.util.List;

@TeleOp
public class TurretSpeed extends LinearOpMode {
    private final CommandScheduler scheduler = new CommandScheduler();
    private enum STATE{INIT,SHOOT}
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Turret turret = new Turret(this,"Turret");
        Command.run(turret.reset());

        ProcessedGamepad driverGamepad = new ProcessedGamepad(gamepad1),
                operatorGamepad = new ProcessedGamepad(gamepad2);
        waitForStart();

        List<LynxModule> lynxModules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule lynxModule : lynxModules)
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        scheduler.schedule(
                new ParallelCommand(
                        turret.update()
                ));

        FSM.FSMBuilder<DuoMode.TeleOpState> fsmBuilder = FSM.<DuoMode.TeleOpState>builder()
                .initial(DuoMode.TeleOpState.INIT);
//                .transition(DuoMode.TeleOpState.INIT, DuoMode.TeleOpState.SHOOTING,operatorGamepad.right_bumper.pressed(),
//                        new InstantCommand(()->turret.setShooterSpeed(4000))
//                );

        FSM<DuoMode.TeleOpState> fsm = fsmBuilder.build(scheduler);

        while(opModeIsActive() && !isStopRequested())
        {
            for (LynxModule lynxModule : lynxModules)
                if (lynxModule.getSerialNumber().isEmbedded()) {
                    lynxModule.clearBulkCache();
                    lynxModule.getBulkData();
                }

           fsm.update();
            operatorGamepad.process();
            telemetry.addData("Current speed",turret.currentSpeed);

            telemetry.update();
        }
    }
}

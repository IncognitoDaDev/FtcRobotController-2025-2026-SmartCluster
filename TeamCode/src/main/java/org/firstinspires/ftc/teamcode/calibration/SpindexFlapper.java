package org.firstinspires.ftc.teamcode.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.ParallelCommand;
import com.smartcluster.oracleftc.utils.ProcessedGamepad;

import org.firstinspires.ftc.teamcode.subsystem.Spindex;

import java.util.concurrent.atomic.AtomicReference;

@Config
@TeleOp (group = "calibration")
public class SpindexFlapper extends LinearOpMode {
    private final CommandScheduler scheduler = new CommandScheduler();

    @Override
    public void runOpMode() throws InterruptedException {

        ProcessedGamepad driverGamepad = new ProcessedGamepad(gamepad1);
        Spindex spindex = new Spindex(this);
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        scheduler.schedule(
                new ParallelCommand(
                        spindex.flapper.update()
                )
        );

        waitForStart();
        while (opModeIsActive()){

            if(driverGamepad.dpad_up.get()) spindex.flapper.setTarget(spindex.flapperUpVal);
            else spindex.flapper.setTarget(spindex.flapperDownVal);

            scheduler.update();

            //telemetry.addData("servo left: ",spindex.servoFlapperLeft.getPosition());
            telemetry.addData("servo right:" ,spindex.servoFlapperRight.getPosition());
            telemetry.update();

            driverGamepad.process();
        }
    }
}

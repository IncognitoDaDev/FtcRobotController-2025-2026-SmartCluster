package org.firstinspires.ftc.teamcode.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.ParallelCommand;
import com.smartcluster.oracleftc.commands.SequentialCommand;
import com.smartcluster.oracleftc.commands.ThreadedCommandScheduler;

import org.firstinspires.ftc.teamcode.subsystem.Turret;

@Config
@TeleOp(group="calibration")
public class turretCalibration extends LinearOpMode {

    private final ThreadedCommandScheduler scheduler = new ThreadedCommandScheduler();
    public static double turretAngle = 90;
    public static double position = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Turret turret = new Turret(this,"Turret");
        CommandScheduler scheduler = new CommandScheduler();
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        scheduler.schedule(
                new SequentialCommand(
                        new ParallelCommand(
                                turret.rotation.reset(),
                                turret.turret.reset()
                        ),
                        new ParallelCommand(
                                turret.rotation.update(),
                                turret.turret.update()
                        )

                ));
        while(opModeIsActive()&&!isStopRequested())
        {
            turret.rotation.setTarget(turretAngle);
            scheduler.update();
            telemetry.update();
            }
        }
}


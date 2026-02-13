package org.firstinspires.ftc.teamcode.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.smartcluster.oracleftc.commands.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystem.Turret;

@Config
@TeleOp(group = "Calibration")
public class HoodCalibration extends LinearOpMode {

    private static final CommandScheduler scheduler = new CommandScheduler();
    public static double targetPosition = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);

        Turret turret = new Turret(this);

        waitForStart();

        scheduler.schedule(turret.hoodact.update());

        while(opModeIsActive()) {
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", turret.hoodact.getTarget());
            telemetry.addData("Instructions", "Use FTC Dashboard to adjust 'targetPosition'");
            telemetry.addData("Range", "0.0 to 1.0");

            turret.hoodact.setTarget(targetPosition);

            scheduler.update();
            telemetry.update();
        }
    }
}
package org.firstinspires.ftc.teamcode.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Spindex;

@Config
@TeleOp(group = "calibration")
public class FlipperDown extends LinearOpMode  {
    public static double Left=0;
    public static double Right=0;
    @Override
    public void runOpMode() throws InterruptedException {
        Spindex spindex = new Spindex(this);
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()){
            spindex.flapperDown(Left,Right);
            telemetry.addData("servo left " , spindex.servoFlapperLeft.getPosition());
            telemetry.addData("servo right" , spindex.servoFlapperRight.getPosition());
            telemetry.update();

        }
    }
}

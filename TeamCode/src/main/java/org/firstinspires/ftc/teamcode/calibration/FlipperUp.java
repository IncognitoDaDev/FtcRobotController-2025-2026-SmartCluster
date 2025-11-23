package org.firstinspires.ftc.teamcode.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Spindex;

@Config
@TeleOp (group = "calibration")
public class FlipperUp extends LinearOpMode {
    public static double Leftup = 0.5;
    public static double Rightup =0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        Spindex spindex = new Spindex(this);
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive()){

            spindex.flapperUp(Leftup,Rightup);
            telemetry.addData("servo left: ",spindex.servoFlapperLeft.getPosition());
            telemetry.addData("servo right: " ,spindex.servoFlapperRight.getPosition());
            telemetry.update();
        }
    }
}

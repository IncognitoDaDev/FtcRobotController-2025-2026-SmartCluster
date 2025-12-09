package org.firstinspires.ftc.teamcode.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.smartcluster.oracleftc.commands.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystem.Spindex;

@TeleOp(group="calibration")
public class dexColorSensorCalibration extends LinearOpMode {

    private final CommandScheduler scheduler = new CommandScheduler();

    public void runOpMode() throws InterruptedException
    {
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Spindex spindex = new Spindex(this);

        waitForStart();
        while(opModeIsActive())
        {
            telemetry.addData("F_Sensor",spindex.IdentifyColor(spindex.rotaryColorSensorF));
            telemetry.addData("L_Sensor",spindex.IdentifyColor(spindex.rotaryColorSensorL));
            telemetry.addData("R_Sensor",spindex.IdentifyColor(spindex.rotaryColorSensorR));

            telemetry.addData("F_Red", spindex.rotaryColorSensorF.red());
            telemetry.addData("F_Green", spindex.rotaryColorSensorF.green());
            telemetry.addData("F_Blue", spindex.rotaryColorSensorF.blue());

            telemetry.addData("L_Red", spindex.rotaryColorSensorL.red());
            telemetry.addData("L_Green", spindex.rotaryColorSensorL.green());
            telemetry.addData("L_Blue", spindex.rotaryColorSensorL.blue());

            telemetry.addData("R_Red", spindex.rotaryColorSensorR.red());
            telemetry.addData("R_Green", spindex.rotaryColorSensorR.green());
            telemetry.addData("R_Blue", spindex.rotaryColorSensorR.blue());

            telemetry.update();
        }
    }

}

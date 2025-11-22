package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.smartcluster.oracleftc.commands.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystem.Spindex;

@TeleOp(group="calibration")
public class SpindexColorSensor extends LinearOpMode {

    private final CommandScheduler scheduler = new CommandScheduler();

    public void runOpMode() throws InterruptedException
    {

        Spindex spindex = new Spindex(this);

        waitForStart();
        while(opModeIsActive())
        {
            telemetry.addData("Red", spindex.rotaryColorSensor.red());
            telemetry.addData("Green", spindex.rotaryColorSensor.green());
            telemetry.addData("Blue", spindex.rotaryColorSensor.blue());
            telemetry.update();
        }
    }

}

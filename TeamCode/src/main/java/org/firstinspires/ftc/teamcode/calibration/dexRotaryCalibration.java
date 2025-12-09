package org.firstinspires.ftc.teamcode.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.ParallelCommand;

import org.firstinspires.ftc.teamcode.subsystem.ColorType;
import org.firstinspires.ftc.teamcode.subsystem.Spindex;

@Config
@TeleOp(group="calibration")
public class dexRotaryCalibration extends LinearOpMode {
    private final CommandScheduler scheduler = new CommandScheduler();

    boolean found = false;

    @Override
    public void runOpMode() throws InterruptedException
    {

        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Spindex spindex = new Spindex(this);
        Command.run(spindex.reset());

        waitForStart();

        scheduler.schedule(
                new ParallelCommand(
                        spindex.update()
                )
        );

        while(opModeIsActive() && !isStopRequested())
        {
            if (Math.abs(spindex.getRotaryPosition() - spindex.rotaryTargetPos) <= spindex.Tolerance)
                spindex.setRotaryPower(0); // Position is good
            else spindex.updateRotaryPosition(); // Position is bad and meh

            if (gamepad1.dpadLeftWasPressed()) spindex.setTarget(spindex.rotaryTargetPos + spindex.ThirdTurn);
            else if (gamepad1.dpadRightWasPressed()) spindex.setTarget(spindex.rotaryTargetPos - spindex.ThirdTurn);
            else if (gamepad1.dpadDownWasPressed()) spindex.setTarget(spindex.rotaryTargetPos + spindex.ThirdTurn/2);
            else if (gamepad1.dpadUpWasPressed()) spindex.setTarget(spindex.rotaryTargetPos - spindex.ThirdTurn/2);


            if (gamepad1.circleWasPressed()) found = spindex.sortAny();
            else if (gamepad1.squareWasPressed()) found = spindex.sortPurple();
            else if (gamepad1.triangleWasPressed()) found = spindex.sortGreen();


            telemetry.addData("EncoderPosition", spindex.rotaryEncoder.getCurrentPosition());
            telemetry.addData("CurrentPosition", spindex.getRotaryPosition());
            telemetry.addData("TargetPosition", spindex.rotaryTargetPos);
            telemetry.addData("ErrorDistance", Math.abs(spindex.rotaryTargetPos-spindex.rotaryEncoder.getCurrentPosition()));

            telemetry.addData("Found desired ball", found);
            telemetry.addData("F_Red", spindex.rotaryColorSensorF.red());
            telemetry.addData("F_Green", spindex.rotaryColorSensorF.green());
            telemetry.addData("F_Blue", spindex.rotaryColorSensorF.blue());

            telemetry.update();
        }
    }


}

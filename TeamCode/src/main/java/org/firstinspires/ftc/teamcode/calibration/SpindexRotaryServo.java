package org.firstinspires.ftc.teamcode.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.ParallelCommand;

import org.firstinspires.ftc.teamcode.subsystem.Spindex;

@Config
@TeleOp(group="calibration")
public class SpindexRotaryServo extends LinearOpMode {
    private final CommandScheduler scheduler = new CommandScheduler();

    static public double Target = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {

        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Spindex spindex = new Spindex(this);
        Command.run(spindex.reset());

        telemetry.addData("CurrentPosition", spindex.rotaryCurrentPos);
        telemetry.addData("Target", spindex.rotaryTargetPos);
        //telemetry.addData("Servo Voltage", spindex.rotaryAnalog.getVoltage());

        telemetry.addData("Red", spindex.rotaryColorSensor.red());
        telemetry.addData("Green", spindex.rotaryColorSensor.green());
        telemetry.addData("Blue", spindex.rotaryColorSensor.blue());

        telemetry.update();

        waitForStart();

        scheduler.schedule(
                new ParallelCommand(
                        spindex.update()
                )
        );

        while(opModeIsActive() && !isStopRequested())
        {
            spindex.setTarget(Target);
            scheduler.update();

            telemetry.addData("CurrentPosition", spindex.getRotaryPosition());
            telemetry.addData("Target", spindex.rotaryTargetPos);
            //telemetry.addData("Servo Voltage", spindex.rotaryAnalog.getVoltage());

            telemetry.addData("Red", spindex.rotaryColorSensor.red());
            telemetry.addData("Green", spindex.rotaryColorSensor.green());
            telemetry.addData("Blue", spindex.rotaryColorSensor.blue());

            telemetry.update();
        }
    }


}

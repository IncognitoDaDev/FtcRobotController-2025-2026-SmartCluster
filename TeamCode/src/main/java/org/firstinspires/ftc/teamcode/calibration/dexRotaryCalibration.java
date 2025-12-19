package org.firstinspires.ftc.teamcode.Calibration;

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

import java.util.function.Supplier;

@Config
@TeleOp(group="calibration")
public class dexRotaryCalibration extends LinearOpMode {
    private final CommandScheduler scheduler = new CommandScheduler();

    Supplier<Boolean> found = ()->false;

    @Override
    public void runOpMode() throws InterruptedException
    {

        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Spindex spindex = new Spindex(this);
        Command.run(spindex.reset());

        waitForStart();


        while(opModeIsActive() && !isStopRequested())
        {
            if (Math.abs(spindex.getPosition() - spindex.rotaryTargetPos) <= Spindex.Tolerance)
                spindex.setRotaryPower(0); // Position is good

            if (gamepad1.dpadLeftWasPressed()) spindex.SwitchMode(-1);
            else if (gamepad1.dpadRightWasPressed()) spindex.NextSpace();


            if (spindex.IdentifyColor(spindex.rotaryColorSensorF) == ColorType.IdentityObject.WALL)
                spindex.cachedSensor.reset();

            spindex.cachedSensor.setFront(spindex.IdentifyColor(spindex.rotaryColorSensorF));
            spindex.cachedSensor.setRight(spindex.IdentifyColor(spindex.rotaryColorSensorR));
            spindex.cachedSensor.setLeft(spindex.IdentifyColor(spindex.rotaryColorSensorL));

            telemetry.addData("CurrentPosition", spindex.getPosition());
            telemetry.addData("TargetPosition", spindex.rotaryTargetPos);
            telemetry.addData("ErrorDistance", Math.abs(spindex.rotaryTargetPos-spindex.getPosition()));

            telemetry.addData("Found desired ball", found);
            telemetry.addData("F_Sensor",spindex.cachedSensor.getFront());
            telemetry.addData("L_Sensor",spindex.cachedSensor.getLeft());
            telemetry.addData("R_Sensor",spindex.cachedSensor.getRight());
            spindex.update();
            telemetry.update();
        }
    }


}

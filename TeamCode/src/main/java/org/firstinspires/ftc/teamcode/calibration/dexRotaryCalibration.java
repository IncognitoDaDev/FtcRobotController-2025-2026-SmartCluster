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
            if (Math.abs(spindex.getPosition() - spindex.rotaryTargetPos) <= spindex.Tolerance)
                spindex.setRotaryPower(0); // Position is good
            else spindex.updateRotaryPosition(); // Position is bad and meh

            if (gamepad1.dpadLeftWasPressed()) spindex.setTarget(spindex.rotaryTargetPos + spindex.ThirdTurn);
            else if (gamepad1.dpadRightWasPressed()) spindex.setTarget(spindex.rotaryTargetPos - spindex.ThirdTurn);


            if (gamepad1.circleWasPressed()) found = spindex.sortAny();
            else if (gamepad1.squareWasPressed()) found = spindex.sortPurple();
            else if (gamepad1.triangleWasPressed()) found = spindex.sortGreen();
            else if (gamepad1.crossWasPressed()) spindex.FixOrientationForIntake();

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

            telemetry.update();
        }
    }


}

package org.firstinspires.ftc.teamcode.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.smartcluster.oracleftc.commands.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystem.ColorType;
import org.firstinspires.ftc.teamcode.subsystem.Spindex_OLD;

@Config
@TeleOp(group="Calibration")
public class dexColorCalibration extends LinearOpMode {

    private final CommandScheduler scheduler = new CommandScheduler();

    public void runOpMode() throws InterruptedException
    {
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Spindex_OLD spindex = new Spindex_OLD(this);

        //Pre-config
        spindex.cachedSensor.setting_WALL = true;


        waitForStart();
        while(opModeIsActive())
        {
            if (spindex.IdentifyColor(spindex.rotaryColorSensorF) == ColorType.IdentityObject.WALL)
                spindex.cachedSensor.reset();

            spindex.cachedSensor.setFront(spindex.IdentifyColor(spindex.rotaryColorSensorF));
            spindex.cachedSensor.setRight(spindex.IdentifyColor(spindex.rotaryColorSensorR));
            spindex.cachedSensor.setLeft(spindex.IdentifyColor(spindex.rotaryColorSensorL));

            telemetry.addData("Last Obj F_Sensor",spindex.cachedSensor.getFront());
            telemetry.addData("Last Obj L_Sensor",spindex.cachedSensor.getLeft());
            telemetry.addData("Last Obj R_Sensor",spindex.cachedSensor.getRight());

            //telemetry.addData("F_Red", spindex.rotaryColorSensorF.red());
            telemetry.addData("F_Green", spindex.rotaryColorSensorF.green());
            telemetry.addData("F_Blue", spindex.rotaryColorSensorF.blue());

            //telemetry.addData("L_Red", spindex.rotaryColorSensorL.red());
            telemetry.addData("L_Green", spindex.rotaryColorSensorL.green());
            telemetry.addData("L_Blue", spindex.rotaryColorSensorL.blue());

            //telemetry.addData("R_Red", spindex.rotaryColorSensorR.red());
            telemetry.addData("R_Green", spindex.rotaryColorSensorR.green());
            telemetry.addData("R_Blue", spindex.rotaryColorSensorR.blue());

            telemetry.update();
        }
    }

}

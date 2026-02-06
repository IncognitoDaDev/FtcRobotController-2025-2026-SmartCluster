package org.firstinspires.ftc.teamcode.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(group="Calibration")
public class ColorCalibration extends LinearOpMode {

    private RevColorSensorV3 sensor_i2c;
    private AnalogInput sensor_Analog;


    @Override
    public void runOpMode() throws InterruptedException {

        sensor_i2c = hardwareMap.get(RevColorSensorV3.class, "rotaryColorSensorF");
        sensor_Analog = hardwareMap.get(AnalogInput.class, "rotaryColorSensorF_Analog");

        waitForStart();
        while(opModeIsActive()){

            NormalizedRGBA data = sensor_i2c.getNormalizedColors();

            telemetry.addData("A", data.alpha*256);
            telemetry.addData("R", data.red*256);
            telemetry.addData("G", data.green*256);
            telemetry.addData("B", data.blue*256);
            telemetry.addData("Distance", sensor_i2c.getDistance(DistanceUnit.MM));

            telemetry.addData("Analog output", sensor_Analog.getVoltage());



            telemetry.update();

        }

    }
}
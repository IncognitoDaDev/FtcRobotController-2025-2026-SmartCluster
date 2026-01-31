package org.firstinspires.ftc.teamcode.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(group="Calibration")
public class ColorI2CCalibration extends LinearOpMode {

    private RevColorSensorV3 frontColorSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        frontColorSensor = hardwareMap.get(RevColorSensorV3.class, "rotaryColorSensorF");

        waitForStart();
        while(opModeIsActive()){

            NormalizedRGBA data = frontColorSensor.getNormalizedColors();
            frontColorSensor.setGain(2);


            telemetry.addData("A", data.alpha*256);
            telemetry.addData("R", data.red*256);
            telemetry.addData("G", data.green*256);
            telemetry.addData("B", data.blue*256);
            telemetry.addData("ARGB", data.toColor());
            telemetry.addData("Distance",frontColorSensor.getDistance(DistanceUnit.MM));

            telemetry.update();

        }

    }
}
package org.firstinspires.ftc.teamcode.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.smartcluster.oracleftc.hardware.OracleLynxVoltageSensor;

import org.firstinspires.ftc.teamcode.subsystem.Robot;

@Config
@TeleOp(group="Calibration")
public class ColorAnalogCalibration extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
//        Storage dex = new Storage(this);
        ColorRangefinder sensor = new ColorRangefinder(hardwareMap.get(RevColorSensorV3.class,"rotaryColorSensorF"));
        final OracleLynxVoltageSensor voltageSensor;


        AnalogInput pin0 = hardwareMap.analogInput.get("rotaryColorSensorF_Analog");

        DigitalChannel pin1 = hardwareMap.digitalChannel.get("rotaryColorSensorF_aux");
        voltageSensor=hardwareMap.getAll(OracleLynxVoltageSensor.class).iterator().next();


        waitForStart();

        sensor.setPin0Analog(ColorRangefinder.AnalogMode.HSV);
        sensor.setPin1Digital(ColorRangefinder.DigitalMode.HSV,100,140);
        while (opModeIsActive()) {
            double coly = pin0.getVoltage()*1000;



            telemetry.addData("voltaj", coly);
            telemetry.addData("NOTHING", coly<1100);
            telemetry.addData("PURPLE", coly>1209);
            telemetry.addData("GREEN", coly>1190 && coly<1208);



//            telemetry.addData("stat", pin1.getState());
//            telemetry.addData("hue", (pin0.getVoltage() / 3.3 * 360));
            telemetry.update();

        }

    }
}
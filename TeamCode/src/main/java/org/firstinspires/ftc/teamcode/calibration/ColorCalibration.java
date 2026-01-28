package org.firstinspires.ftc.teamcode.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.ColorType;
import org.firstinspires.ftc.teamcode.subsystem.Storage;

@Config
@TeleOp(group="Calibration")
public class ColorCalibration extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
//        Storage dex = new Storage(this);

        DigitalChannel pin0 = hardwareMap.digitalChannel.get("rotaryColorSensorF_Purple");
        DigitalChannel pin1 = hardwareMap.digitalChannel.get("rotaryColorSensorF_Green");

        waitForStart();
        while(opModeIsActive()){

            telemetry.addData("Is purple?", pin0.getState());
            telemetry.addData("Is green?", pin1.getState());
//            telemetry.addData("IdentifyObj", dex.identifyObj());

            telemetry.update();

        }

    }
}
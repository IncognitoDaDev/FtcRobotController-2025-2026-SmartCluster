package org.firstinspires.ftc.teamcode.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.ColorType;
import org.firstinspires.ftc.teamcode.subsystem.Storage;

@Config
@TeleOp(group="Calibration")
public class ColorCalibration extends LinearOpMode {

    private RevColorSensorV3 frontColorSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        Storage dex = new Storage(this);

        frontColorSensor = hardwareMap.get(RevColorSensorV3.class, "rotaryColorSensorF");


        waitForStart();
        while(opModeIsActive()){
            NormalizedRGBA data = frontColorSensor.getNormalizedColors();

            Storage.ArtifactColor[] order = {Storage.ArtifactColor.PURPLE, Storage.ArtifactColor.PURPLE, Storage.ArtifactColor.GREEN};
            Storage.ArtifactColor[] order2 = {Storage.ArtifactColor.GREEN, Storage.ArtifactColor.PURPLE, Storage.ArtifactColor.PURPLE};


            for(int i = 0; i < 3; i++)
            {
                dex.sort(order2[i]);
                dex.flapperUp();
            }

            telemetry.addData("A", data.alpha*256);
            telemetry.addData("R", data.red*256);
            telemetry.addData("G", data.green*256);
            telemetry.addData("B", data.blue*256);
            telemetry.addData("ARGB", data.toColor());

            telemetry.update();

        }

    }
}
package org.firstinspires.ftc.teamcode.Calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.Turret;
@Config
@TeleOp(group="calibration")
public class Hood_calibration extends LinearOpMode {
    public Servo leftHood,rightHood;
    public void runOpMode() throws InterruptedException {
        Turret turret = new Turret(this,"Turret");
        leftHood = hardwareMap.get(Servo.class,"leftHood");
        rightHood = hardwareMap.get(Servo.class,"rightHood");

        waitForStart();
        while(opModeIsActive())
        {

            if (gamepad2.right_bumper) {      // R1
                        leftHood.setPosition(0);
                        rightHood.setPosition(0);
            }
            else {
                leftHood.setPosition(1);
                rightHood.setPosition(1);
            }

            telemetry.update();
        }
    }

}

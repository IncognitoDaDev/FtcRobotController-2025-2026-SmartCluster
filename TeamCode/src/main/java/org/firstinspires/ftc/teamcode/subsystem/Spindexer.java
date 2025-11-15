package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Spindexer extends LinearOpMode {

    private Servo sservo1, sservo2;

    @Override
    public void runOpMode() throws InterruptedException {

        sservo1 = hardwareMap.get(Servo.class, "sservo1");
        sservo2 = hardwareMap.get(Servo.class, "sservo2");


        double servo1Pos = 0.5;
        double servo2Pos = 0.5;

        waitForStart();

        while (opModeIsActive()) {


            if (gamepad2.circle) {
               sservo1.setPosition(servo1Pos);
            }

            if (gamepad2.triangle) {
                sservo2.setPosition(servo2Pos);
            }


            sservo1.setPosition(servo1Pos);
            sservo2.setPosition(servo2Pos);

            telemetry.addData("Servo 1 Position", servo1Pos);
            telemetry.addData("Servo 2 Position", servo2Pos);
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.smartcluster.oracleftc.utils.ProcessedGamepad;

import org.firstinspires.ftc.teamcode.subsystem.Intake;


public class SoloMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        ProcessedGamepad driverGamepad = new ProcessedGamepad(gamepad1);

        Intake intake = new Intake(this);

       while(opModeIsActive())
        {
            double intakePower = 0;

            if (gamepad2.right_bumper) {      // R1
                intake.intake();

            }
            else if (gamepad2.left_bumper) {  // L1
               intake.intake();
            }
            else {
               intake.reset();
            }

            telemetry.addData("intakePower:", intakePower);
            telemetry.update();
            sleep(20);
        }

    }
}

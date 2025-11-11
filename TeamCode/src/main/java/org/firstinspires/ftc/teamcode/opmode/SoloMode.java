package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.smartcluster.oracleftc.utils.ProcessedGamepad;


public class SoloMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        ProcessedGamepad driverGamepad = new ProcessedGamepad(gamepad1);

        while(opModeIsActive())
        {
            // do stuff
        }

    }
}

package org.firstinspires.ftc.teamcode.Calibration;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.smartcluster.oracleftc.utils.ProcessedGamepad;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Spindex;

@Config
@TeleOp(group="calibration")
public class intakeCalibration extends LinearOpMode {

        @Override
        public void runOpMode() throws InterruptedException {
            waitForStart();
            ProcessedGamepad driverGamepad = new ProcessedGamepad(gamepad1);
            Spindex soindex = new Spindex(this);
            Intake intake = new Intake(this);

            while(opModeIsActive())
            {

                if (gamepad2.right_bumper) {      // R1
                    intake.intake();

                }
                else {
                    intake.reset();
                }

                telemetry.addData("intakePower:", intake.intakeMotor.getCurrentPosition());
                telemetry.update();
            }

        }
    }
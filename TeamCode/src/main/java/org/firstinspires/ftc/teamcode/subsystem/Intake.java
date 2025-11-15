package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="IntakeControl")
public class Intake extends LinearOpMode {

    public DcMotor intakeMotor;


    @Override
    public void runOpMode() throws InterruptedException {


        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            double intakePower = 0;

            if (gamepad2.right_bumper) {      // R1
                intakePower = 1.0;
            }
            else if (gamepad2.left_bumper) {  // L1
                intakePower = -1.0;
            }
            else {
                intakePower = 0;
            }

            intakeMotor.setPower(intakePower);

            telemetry.addData("intakePower:", intakePower);
            telemetry.update();
            sleep(20);
        }

    }

}

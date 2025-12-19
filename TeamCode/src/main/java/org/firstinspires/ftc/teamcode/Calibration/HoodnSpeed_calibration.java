package org.firstinspires.ftc.teamcode.Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@Config
@TeleOp(group="calibration")
public class HoodnSpeed_calibration extends LinearOpMode {
    public DcMotorEx turretUp,turretDown;
    public Servo hood;

    public static double highvelocity = 5500;
    public static double lowvelocity = 4000;
    public static double currentVelocity = highvelocity;
    public static double position = 1;

    double F = 0;
    double P = 0;
    double stepIndex = 1;
    double[] stepSize = {100,10,1,0.1,0.01,0.0001};
    @Override
    public void runOpMode() throws InterruptedException {
        hood = hardwareMap.get(Servo.class,"rightHood");
        turretDown = hardwareMap.get(DcMotorEx.class,"turretDown");
        turretUp = hardwareMap.get(DcMotorEx.class,"turretUp");
        turretUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretUp.setDirection(DcMotorSimple.Direction.REVERSE);
        turretDown.setDirection(DcMotorSimple.Direction.FORWARD);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        turretUp.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        turretDown.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);

        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.xWasPressed()){
                if(currentVelocity == highvelocity){
                    currentVelocity = lowvelocity;
                }
                else{
                    currentVelocity = highvelocity;
                }
            }
            if(gamepad1.squareWasPressed())
                stepIndex = (stepIndex+1)% stepSize.length;

            if(gamepad1.dpadRightWasPressed())F+=stepIndex;
            if(gamepad1.dpadLeftWasPressed())F-=stepIndex;

            if(gamepad1.dpadUpWasPressed())P+=stepIndex;
            if(gamepad1.dpadDownWasPressed())P-=stepIndex;

            turretUp.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
            turretDown.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
            if (gamepad1.leftBumperWasPressed()){hood.setPosition(position);position+= 0.1;}

            turretUp.setVelocity(currentVelocity*8192/6000);
            turretDown.setPower(turretUp.getPower());

            double currentSpeed = turretUp.getVelocity();
            double error = currentVelocity - currentSpeed;

            telemetry.addData("Target velocity", currentVelocity);
            telemetry.addData("Current velocity", currentSpeed);
            telemetry.addData("Error", error);

            telemetry.addData("F",F);
            telemetry.addData("P",P);
            telemetry.addData("Step index", stepIndex);
            telemetry.update();

        }

    }
}
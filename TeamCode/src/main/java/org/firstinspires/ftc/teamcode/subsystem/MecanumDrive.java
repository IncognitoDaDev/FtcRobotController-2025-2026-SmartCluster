package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.Command.CommandBuilder;
import com.smartcluster.oracleftc.utils.ProcessedGamepad;

public class MecanumDrive {
    private DcMotor frontRight, frontLeft, backRight, backLeft;
    private HardwareMap hardwareMap;
    public MecanumDrive(OpMode opMode,HardwareMap hardwareMap)
    {
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Command drive(ProcessedGamepad gamepad)
    {
        return new CommandBuilder()
                .update(()->{
                    double y = -gamepad.left_stick.get().y; // Remember, Y stick is reversed!
                    double x = gamepad.left_stick.get().x; // Counteract imperfect strafing

                    if(gamepad.left_trigger.get() < 0.1)
                    {
                        y/=2;
                        x/=2;
                    }

                    double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
                    double frontLeftPower = (y + x) / denominator;
                    double backLeftPower = (y - x) / denominator;
                    double frontRightPower = (y - x) / denominator;
                    double backRightPower = (y + x) / denominator;

                    frontRight.setPower(frontRightPower);
                    frontLeft.setPower(frontLeftPower);
                    backLeft.setPower(backLeftPower);
                    backRight.setPower(backRightPower);
                })
                .build();
    }



}

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
    double baseSpeed = 1.0;
    private HardwareMap hardwareMap;
    public MecanumDrive(OpMode opMode,HardwareMap hardwareMap)
    {
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Command drive(ProcessedGamepad gamepad)
    {
        return new CommandBuilder()
                .update(()->{
                    ProcessedGamepad.Joystick.JoystickData leftStick = gamepad.left_stick.get();
                    ProcessedGamepad.Joystick.JoystickData rightStick = gamepad.right_stick.get();

                    double boost = (gamepad.left_bumper.get() || gamepad.right_bumper.get() ? 1 : 0.3);

                    double rx = rightStick.x * boost;
                    double x = leftStick.x * 1.0 * boost;
                    double y = -leftStick.y * boost;

                    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                    double frontLeftPower = (y + x + rx) / denominator * baseSpeed;
                    double backLeftPower = (y - x + rx) / denominator * baseSpeed;
                    double frontRightPower = (y - x - rx) / denominator * baseSpeed;
                    double backRightPower = (y + x - rx) / denominator * baseSpeed;

                    frontRight.setPower(frontRightPower);
                    frontLeft.setPower(frontLeftPower);
                    backLeft.setPower(backLeftPower);
                    backRight.setPower(backRightPower);
                })
                .build();
    }



}

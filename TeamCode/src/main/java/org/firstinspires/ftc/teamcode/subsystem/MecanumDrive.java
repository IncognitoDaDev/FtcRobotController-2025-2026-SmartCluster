package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.Command.CommandBuilder;
import com.smartcluster.oracleftc.hardware.OracleLynxVoltageSensor;
import com.smartcluster.oracleftc.utils.ProcessedGamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class MecanumDrive {
    private DcMotor frontRight, frontLeft, backRight, backLeft;
    public final OracleLynxVoltageSensor voltageSensor;
    public double baseSpeed = 1.0;

    public MecanumDrive(HardwareMap hardwareMap)
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

        voltageSensor = hardwareMap.getAll(OracleLynxVoltageSensor.class).iterator().next();
        voltageSensor.setPolicy(OracleLynxVoltageSensor.OracleLynxVoltageSensorPolicy.CACHED);
        voltageSensor.setVoltageCacheFreshness(300);

        setDriveMode(DriveMode.NORMAL);
    }

    public Command drive(ProcessedGamepad gamepad)
    {
        return new CommandBuilder()
                .update(()->{
                    ProcessedGamepad.Joystick.JoystickData leftStick = gamepad.left_stick.get();
                    ProcessedGamepad.Joystick.JoystickData rightStick = gamepad.right_stick.get();

                    double boost = (gamepad.left_bumper.get() || gamepad.right_bumper.get() ? 1 : 0);

                    double rx = rightStick.x * boost;
                    double x = leftStick.x * 1.0 * boost;
                    double y = -leftStick.y * boost;

                    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                    double frontLeftPower = (y + x + rx) / denominator * baseSpeed;
                    double backLeftPower = (y - x + rx) / denominator * baseSpeed;
                    double frontRightPower = (y - x - rx) / denominator * baseSpeed;
                    double backRightPower = (y + x - rx) / denominator * baseSpeed;

                    setMotorPowers(frontRightPower, backRightPower, frontLeftPower, backLeftPower);
                })
                .build();
    }

    enum DriveMode
    {
        NORMAL,
        PREFIRE,
        PARKING
    }

    public void setDriveMode(DriveMode mode)
    {
        switch(mode)
        {
            case NORMAL: baseSpeed = 1.0; break;
            case PREFIRE: baseSpeed = 0.7; break;
            case PARKING: baseSpeed = 0.4; break;
        }
    }


    public void setMotorPowers(double frontRightPower, double backRightPower, double frontLeftPower,
                               double backLeftPower) {

        double voltage = voltageSensor.getVoltage();
        frontRight.setPower(frontRightPower*(12.0/voltage));
        backRight.setPower(backRightPower*(12.0/voltage));
        frontLeft.setPower(frontLeftPower*(12.0/voltage));
        backLeft.setPower(backLeftPower*(12.0/voltage));
    }




}

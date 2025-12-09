package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;

public class Intake extends Subsystem {

    public DcMotor intakeMotor;
    public double intakePower = 0.72;
    public double slowlyintakePower = 0.5;

    public double fastIntakePower = 0.88;
    public Intake(OpMode mode)
    {
        super(mode);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void reset() {
        intakeMotor.setPower(0);
    }

    public void setTarget(double power) {
        intakeMotor.setPower(power);
    }

    public void intake() {
        intakeMotor.setPower(intakePower);
    }

    public void slowlyIntake() {
        intakeMotor.setPower(slowlyintakePower);
    }

    public void fastIntake() {
        intakeMotor.setPower(fastIntakePower);
    }

//            double intakePower = 0;
//
//            if (gamepad2.right_bumper) {      // R1
//                intakeMotor.setPower(intakePower);
//            }
//            else if (gamepad2.left_bumper) {  // L1
//                intakeMotor.setPower(intakePower);
//            }
//            else {
//               intakeMotor.setPower(0);
//            }
//
//            intakeMotor.setPower(intakePower);
//
//            telemetry.addData("intakePower:", intakePower);
//            telemetry.update();
//            sleep(20);
        }


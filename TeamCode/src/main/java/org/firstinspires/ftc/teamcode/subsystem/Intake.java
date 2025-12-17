package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;

public class Intake extends Subsystem {

    public DcMotorImplEx intakeMotor;
    public double intakePower = 0.72;

    public Intake(OpMode mode)
    {
        super(mode);
        intakeMotor = hardwareMap.get(DcMotorImplEx.class, "intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void reset() {
        intakeMotor.setPower(0);
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public void intake() {
        intakeMotor.setPower(intakePower);
    }


}

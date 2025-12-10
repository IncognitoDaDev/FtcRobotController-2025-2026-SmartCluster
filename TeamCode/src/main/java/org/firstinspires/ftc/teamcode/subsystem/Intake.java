package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;

public class Intake extends Subsystem {

    public DcMotor intakeMotor;
    public double intakePower = 0.85;

    public Intake(OpMode mode)
    {
        super(mode);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void reset() {
        intakeMotor.setPower(0);
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public void Intake() {
        intakeMotor.setPower(intakePower);
    }


}

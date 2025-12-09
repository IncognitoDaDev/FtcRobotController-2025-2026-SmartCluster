package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;

public class Intake extends Subsystem {
    private DcMotorImplEx intake;
    public Intake(OpMode opMode) {
        super(opMode);
        intake = hardwareMap.get(DcMotorImplEx.class,"intake");

    }
    public void On(){
            intake.setPower(0.7);
    }
    public void Reset(){
        intake.setPower(0);
    }
}

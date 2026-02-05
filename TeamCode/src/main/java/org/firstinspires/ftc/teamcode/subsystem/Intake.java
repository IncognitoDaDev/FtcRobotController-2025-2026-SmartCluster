package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.hardware.OmegaPowerCollector;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;
import com.smartcluster.oracleftc.hardware.wrappers.OmegaDcMotorImplEx;

public class Intake extends Subsystem {


    private final OmegaDcMotorImplEx intakeMotor;
    public Intake(OpMode opMode, OmegaPowerCollector powerCollector) {
        super(opMode);

        intakeMotor = new OmegaDcMotorImplEx(hardwareMap.get(DcMotorImplEx.class, "intakeMotor"), powerCollector, true);
    }

    public Command intake()
    {
        return new InstantCommand(()-> intakeMotor.setPower(1.0));
    }
    public Command outtake()
    {
        return new InstantCommand(()->intakeMotor.setPower(-1.0));
    }

    public Command stop()
    {
        return new InstantCommand(()->intakeMotor.setPower(0));
    }


}

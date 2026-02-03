package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.hardware.OmegaPowerCollector;
import com.smartcluster.oracleftc.hardware.motor.OracleDcMotorImplEx;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;

public class Intake extends Subsystem {


    private final OracleDcMotorImplEx intakeMotor;
    public Intake(OpMode opMode, OmegaPowerCollector powerCollector) {
        super(opMode);

        intakeMotor = (OracleDcMotorImplEx) hardwareMap.get(DcMotorImplEx.class, "intakeMotor");
        intakeMotor.setDestination(powerCollector, true); // port 3
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

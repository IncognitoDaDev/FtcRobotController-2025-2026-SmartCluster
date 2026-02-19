package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.hardware.OracleLynxVoltageSensor;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;
import com.smartcluster.oracleftc.hardware.subsystem.SubsystemFlavor;

public class Intake extends Subsystem {

    private final DcMotorImplEx intakeMotor;
    private final OracleLynxVoltageSensor voltageSensor;
    public Intake(OpMode opMode) {
        super(opMode);

        intakeMotor=hardwareMap.get(DcMotorImplEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        voltageSensor=hardwareMap.getAll(OracleLynxVoltageSensor.class).iterator().next();
    }

    public Command intake()
    {
        return new InstantCommand(()->intakeMotor.setPower(Robot.nominalVoltage/voltageSensor.getVoltage()));
    }
    public Command outake()
    {
        return new InstantCommand(()->intakeMotor.setPower(-Robot.nominalVoltage/voltageSensor.getVoltage()));
    }

    public Command stop()
    {
        return new InstantCommand(()->intakeMotor.setPower(0));
    }

    @Override
    public SubsystemFlavor flavor() {
        return SubsystemFlavor.ExpansionHubOnly;
    }

}

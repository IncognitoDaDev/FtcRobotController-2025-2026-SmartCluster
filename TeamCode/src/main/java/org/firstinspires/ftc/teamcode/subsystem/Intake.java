package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

        intakeMotor = hardwareMap.get(DcMotorImplEx.class, "intake");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        voltageSensor = hardwareMap.getAll(OracleLynxVoltageSensor.class).iterator().next();
    }

    public Command intake() {
        return new InstantCommand(() ->
                intakeMotor.setPower(1)
        );
    }

    public Command outake() {
        return new InstantCommand(() ->
                intakeMotor.setPower(-1)
        );
    }

    public Command stop() {
        return new InstantCommand(() ->
                intakeMotor.setPower(0)
        );
    }

    /**
     * Set intake to run at a constant passive power
     * Useful for keeping rings moving slowly during calibration
     */
    public void setPassivePower(double power) {
        intakeMotor.setPower(power);
    }


    @Override
    public SubsystemFlavor flavor() {
        return SubsystemFlavor.ExpansionHubOnly;
    }
}
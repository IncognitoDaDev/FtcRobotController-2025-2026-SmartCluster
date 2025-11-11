package com.smartcluster.oracleftc.hardware.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Subsystem {

    protected Telemetry telemetry;
    protected HardwareMap hardwareMap;
    public Subsystem(OpMode opMode) {
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;
    }

    public SubsystemFlavor flavor()
    {
        return SubsystemFlavor.Mixed;
    }
}

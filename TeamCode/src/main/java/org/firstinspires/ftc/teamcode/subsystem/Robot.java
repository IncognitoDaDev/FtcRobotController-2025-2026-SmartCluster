package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.ParallelCommand;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;

public class Robot {
    private final OpMode opMode;
    public final MecanumDrive mecanumDrive;
    public final Intake intake;
    public final Spindex spinDex;
    public final Turret turret;

    public Robot(OpMode mode)
    {

        this.opMode = mode;
        this.spinDex = new Spindex(mode);
        this.intake = new Intake(mode);
        this.mecanumDrive = new MecanumDrive(mode.hardwareMap, new Pose2d(0, 0, 0));
        this.turret = new Turret(mode,"Turret");

    }

    public Command reset()
    {
        return new ParallelCommand(
            turret.hood.reset(),
            spinDex.reset()
            //turret.reset()
        );
    }

    public Command update()
    {
        return new ParallelCommand(
                turret.hood.update(),
                turret.update(),
                spinDex.flapper.update()
                //spinDex.update()
        );
    }

}

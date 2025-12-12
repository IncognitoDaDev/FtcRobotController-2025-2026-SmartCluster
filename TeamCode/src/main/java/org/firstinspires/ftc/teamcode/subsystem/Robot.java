package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.ParallelCommand;

public class Robot {
    private final OpMode opMode;
    public final MecanumDrive mecanumDrive;
    public final Intake intake;
    public final Spindex spinDex;
    public final Turret turret;

    public Robot(OpMode mode)
    {

        this.opMode = mode;

        this.intake = new Intake(mode);
        this.mecanumDrive = new MecanumDrive(mode.hardwareMap, new Pose2d(0, 0, 0));
        this.spinDex = new Spindex(mode);
        this.turret = new Turret(mode,"Turret");

    }

    public Command reset()
    {
        return new ParallelCommand(
            turret.hood.reset(),
            spinDex.reset(),
            turret.reset()
        );
    }

    public Command update()
    {
        return new ParallelCommand(
                turret.hood.update(),
                turret.update(),
                spinDex.update()
        );
    }

}

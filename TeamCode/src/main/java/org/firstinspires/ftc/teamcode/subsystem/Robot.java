package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.ParallelCommand;

public class Robot {
    private final OpMode opMode;
    public final MecanumDrive mecanumDrive;
    public final Intake intake;
    public final Spindex spindex;
    public final Turret turret;

    public Robot(OpMode mode)
    {

        this.opMode = mode;
        this.spindex = new Spindex(mode);
        this.intake = new Intake(mode);
        this.mecanumDrive = new MecanumDrive(mode.hardwareMap, new Pose2d(0, 0, 0));
        this.turret = new Turret(mode,"Turret");

    }

    public Command reset()
    {
        return new ParallelCommand(
            turret.hood.reset(),
            spindex.reset()
            //turret.reset()
        );
    }

    public Command update()
    {
        return new ParallelCommand(
                turret.hood.update(),
//                turret.update(),
                spindex.flapper.update(),
                turret.ppUpdate(mecanumDrive.localizer)
                //spinDex.update()
        );
    }

}

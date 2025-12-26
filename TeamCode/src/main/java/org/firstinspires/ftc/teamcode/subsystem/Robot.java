package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.ParallelCommand;
@Config
public class Robot {
    public static double nominalVoltage=12.0;
    private final OpMode opMode;
    public final Turret flywheel;
    public final MecanumDrive drive;
    public final Intake intake;
    public final Spindex_OLD spindex;
    public final Turret turret;

    public Robot(OpMode mode)
    {

        this.opMode = mode;
        this.flywheel=new Turret(mode);
        this.spindex = new Spindex_OLD(mode);
        this.intake = new Intake(mode);
        this.drive = new MecanumDrive(mode.hardwareMap, new Pose2d(0, 0, 0));
        this.turret = new Turret(mode);

    }

    public Command reset()
    {
        return new ParallelCommand(
            turret.hood.reset(),
            spindex.flapper.reset()
        );
    }

    public Command update()
    {
        return new ParallelCommand(
                turret.update(),
                spindex.update()
        );
    }

}

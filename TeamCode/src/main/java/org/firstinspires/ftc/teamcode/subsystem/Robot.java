package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.ParallelCommand;

public class Robot {
    private final OpMode opMode;
    public final MecanumDrive mecanumDrive;

    public Robot(OpMode mode)
    {
        this.opMode = mode;
        this.mecanumDrive = new MecanumDrive(mode.hardwareMap, new Pose2d(0,0,0));
    }

    public Command reset()
    {
        return new ParallelCommand(

        );
    }

    public Command update()
    {
        return new ParallelCommand(

        );
    }

}

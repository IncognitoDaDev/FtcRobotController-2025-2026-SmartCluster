package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.ParallelCommand;

public class Robot {
    private final OpMode opMode;

    public Robot(OpMode mode)
    {
        this.opMode = mode;
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

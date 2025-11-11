package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.fsm.FSM;
import com.smartcluster.oracleftc.utils.ProcessedGamepad;


@Config
@TeleOp(name="DuoMode")
public class DuoMode extends LinearOpMode {
    private final CommandScheduler scheduler = new CommandScheduler();

    @Override
    public void runOpMode() throws InterruptedException
    {
        waitForStart();
        ProcessedGamepad driverGamepad = new ProcessedGamepad(gamepad1),
                        operatorGamepad = new ProcessedGamepad(gamepad2);

        while(opModeIsActive())
        {
            // do stuff
        }

        while(opModeIsActive() && !isStopRequested())
        {
            driverGamepad.process();
            operatorGamepad.process();

        }
    }
}

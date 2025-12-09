package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.ParallelCommand;
import com.smartcluster.oracleftc.hardware.OracleLynxVoltageSensor;
import com.smartcluster.oracleftc.utils.ProcessedGamepad;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

import java.util.List;


@Config
@TeleOp(name="DuoMode")
public class DuoMode extends LinearOpMode {
    private final CommandScheduler scheduler = new CommandScheduler();

    @Override
    public void runOpMode() throws InterruptedException
    {
        ProcessedGamepad driverGamepad = new ProcessedGamepad(gamepad1),
                        operatorGamepad = new ProcessedGamepad(gamepad2);
        Robot robot = new Robot(this);

        waitForStart(); // STARTING POINT


        scheduler.schedule(
                new ParallelCommand(
                        robot.mecanumDrive.drive(driverGamepad)
                )
        );

        while(opModeIsActive())
        {

            telemetry.update();

            driverGamepad.process();
            operatorGamepad.process();
        }
    }
}

package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.ParallelCommand;
import com.smartcluster.oracleftc.fsm.FSM;
import com.smartcluster.oracleftc.utils.ProcessedGamepad;

import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Robot;


@Config
@TeleOp(name="SoloMode")
public class SoloMode extends LinearOpMode {
    private final CommandScheduler scheduler = new CommandScheduler();

    enum TeleOpState
    {
        INIT,
        IDLE,
        PREFIRE,
        PARKING
    }
    @Override
    public void runOpMode() throws InterruptedException
    {
        ProcessedGamepad driverGamepad = new ProcessedGamepad(gamepad1);
        Robot robot = new Robot(this);

        Command.run(robot.reset());
        waitForStart(); // STARTING POINT


        scheduler.schedule(
                new ParallelCommand(
                        robot.mecanumDrive.drive(driverGamepad)
                )
        );

        FSM.FSMBuilder<TeleOpState> fsmBuilder =  FSM.<TeleOpState>builder()
                .initial(TeleOpState.IDLE);

        FSM<TeleOpState> fsm = fsmBuilder.build(scheduler);

        while(opModeIsActive())
        {
            if(driverGamepad.dpad_down.get()) robot.mecanumDrive.setDriveMode(MecanumDrive.DriveMode.PREFIRE);
            if(driverGamepad.dpad_down.get()) robot.mecanumDrive.setDriveMode(MecanumDrive.DriveMode.PARKING);
            if(driverGamepad.dpad_up.get()) robot.mecanumDrive.setDriveMode(MecanumDrive.DriveMode.NORMAL);

            telemetry.update();

            fsm.update();
            driverGamepad.process();
        }
    }
}

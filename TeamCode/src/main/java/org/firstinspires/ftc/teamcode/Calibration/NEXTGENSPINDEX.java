package org.firstinspires.ftc.teamcode.Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.ParallelCommand;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Spindex;

@Config
@TeleOp(group="calibration")
public class NEXTGENSPINDEX extends LinearOpMode {
    private final CommandScheduler scheduler = new CommandScheduler();

    Boolean canI = false;

    @Override
    public void runOpMode() throws InterruptedException
    {

        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Spindex spindex = new Spindex(this);
        Intake intake = new Intake(this);
        Command.run(spindex.reset());

        scheduler.schedule(
                new ParallelCommand(
                        spindex.update()
//                        robot.turret.rotation.update()
                ));

        waitForStart();


        while(opModeIsActive() && !isStopRequested())
        {
            if (canI) {
                if (gamepad1.dpadLeftWasPressed()) spindex.moveVoid(-spindex.ThirdTurn);
                else if (gamepad1.dpadRightWasPressed()) spindex.moveVoid(spindex.ThirdTurn);

                scheduler.update();
            }

            if (gamepad1.crossWasPressed()) canI = !canI;

            telemetry.addData("CurrentPosition", spindex.getPosition());
            telemetry.addData("TargetPosition", spindex.rotaryTargetPos);
            telemetry.addData("ErrorDistance", Math.abs(spindex.rotaryTargetPos-spindex.getPosition()));

            telemetry.update();
        }
    }


}

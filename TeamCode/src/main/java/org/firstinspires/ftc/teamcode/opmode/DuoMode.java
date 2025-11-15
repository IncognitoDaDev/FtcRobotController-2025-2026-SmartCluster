package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.ParallelCommand;
import com.smartcluster.oracleftc.hardware.OracleLynxVoltageSensor;
import com.smartcluster.oracleftc.utils.ProcessedGamepad;

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
        List<LynxModule> modules = hardwareMap.getAll(LynxModule.class);

        Command.run(robot.reset());
        waitForStart(); // STARTING POINT

        List<LynxModule> lynxModules = hardwareMap.getAll(LynxModule.class);
        for(LynxModule lynxModule: lynxModules)
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        List<OracleLynxVoltageSensor> voltageSensors =hardwareMap.getAll(OracleLynxVoltageSensor.class);
        for (OracleLynxVoltageSensor voltageSensor :
                voltageSensors) {
            voltageSensor.setPolicy(OracleLynxVoltageSensor.OracleLynxVoltageSensorPolicy.CACHED);
            voltageSensor.setVoltageCacheFreshness(100);

        }


        scheduler.schedule(
                new ParallelCommand(
                        robot.mecanumDrive.drive(driverGamepad)
                )
        );

        while(opModeIsActive())
        {
            for(LynxModule lynxModule: lynxModules)
                if(lynxModule.getSerialNumber().isEmbedded())
                {
                    lynxModule.clearBulkCache();
                    lynxModule.getBulkData();

                }

            telemetry.addData("state", 1);
            telemetry.update();


            driverGamepad.process();
            operatorGamepad.process();
        }
    }
}

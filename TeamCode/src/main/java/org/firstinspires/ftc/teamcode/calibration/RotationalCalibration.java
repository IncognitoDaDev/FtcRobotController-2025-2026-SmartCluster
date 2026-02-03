package org.firstinspires.ftc.teamcode.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

import java.util.List;

@Config
@TeleOp(group = "Calibration")
public class RotationalCalibration extends LinearOpMode {

    private static final CommandScheduler scheduler = new CommandScheduler();
    public static double target = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);
//        Turret turret = new Turret(this);
        Robot robot = new Robot(this, false);
        waitForStart();
        Command.run(robot.turret.reset());
        List<LynxModule> lynxModules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule lynxModule : lynxModules)
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        scheduler.schedule(robot.turret.update());

        while (opModeIsActive()) {

            robot.powerManager.read();

            telemetry.addData("target", target);
            telemetry.addData("target", 180.0);
            telemetry.addData("Current velocity", robot.turret.turret.getPosition());

            if (gamepad1.cross) robot.turret.turret.setTarget(180);
            if (gamepad1.square) robot.turret.turret.setTarget(0);


            scheduler.update();
            telemetry.update();

            for (LynxModule lynxModule : lynxModules)
            {
                lynxModule.clearBulkCache();
                lynxModule.getBulkData();
            }
        }
    }
}

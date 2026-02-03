package org.firstinspires.ftc.teamcode.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.smartcluster.oracleftc.commands.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

import java.util.List;

@Config
@TeleOp(group = "Calibration")
public class FlywheelCalibration extends LinearOpMode {

    private static final CommandScheduler scheduler = new CommandScheduler();
    public static double targetVelocity=0;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        telemetry.setMsTransmissionInterval(100);
//        Turret flywheel =new Turret(this);
        Robot robot = new Robot(this, false);
        waitForStart();

        List<LynxModule> lynxModules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule lynxModule : lynxModules)
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        scheduler.schedule(robot.flywheel.update());

        while(opModeIsActive())
        {
            robot.powerManager.read();

            telemetry.addData("targetVelocity", targetVelocity);
            telemetry.addData("Current velocity", robot.flywheel.getCurrentVelocity());
            robot.flywheel.setTargetVelocity(targetVelocity);
            scheduler.update();
            telemetry.update();
        }
    }
}

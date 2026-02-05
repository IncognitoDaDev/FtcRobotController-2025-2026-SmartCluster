package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorEncoderPositionCommand;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.ParallelCommand;
import com.smartcluster.oracleftc.commands.SequentialCommand;
import com.smartcluster.oracleftc.hardware.OmegaPowerCollector;
import com.smartcluster.oracleftc.hardware.wrappers.DcMotor;

import java.util.HashMap;
import java.util.List;

@Config
public class Robot {
    public static double nominalVoltage=10.0;
    private final OpMode opMode;

    public final OmegaPowerCollector powerManager;
    private final boolean color;
    public final Turret flywheel;
    public final MecanumDrive drive;
    public final Intake intake;
    public final Storage storage;
    public final Turret turret;

    public final Limelight cam;

    List<LynxDcMotorController> DcMotorController;
    List<LynxServoController> ServoController;

    public final List<LynxModule> lynxModules;
    public LynxModule ControlHub, ExpansionHub;

    public Robot(OpMode mode, boolean color)
    {
        this.opMode = mode;

        lynxModules = opMode.hardwareMap.getAll(LynxModule.class);

        for (LynxModule lynxModule : lynxModules)
            if (lynxModule.getSerialNumber().isEmbedded()) {
                ControlHub = lynxModule;
            }
        for (LynxModule lynxModule : lynxModules)
            if (!lynxModule.getSerialNumber().isEmbedded()) {
                ExpansionHub = lynxModule;
            }

        ControlHub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
        ExpansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);

        DcMotorController = opMode.hardwareMap.getAll(LynxDcMotorController.class);
        ServoController = opMode.hardwareMap.getAll(LynxServoController.class);

        this.powerManager = new OmegaPowerCollector(mode) {
            @Override
            public void read()
            {
                try {
                    new LynxGetMotorEncoderPositionCommand(ControlHub, 3).send();

                    new LynxGetMotorEncoderPositionCommand(ExpansionHub, 0).send();
                    new LynxGetMotorEncoderPositionCommand(ExpansionHub, 1).send();
//                    new LynxGetMotorEncoderPositionCommand(ExpansionHub, 3).send();
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                } catch (LynxNackException e) {
                    throw new RuntimeException(e);
                }

                // Use only one servo of two-servo set for position
                ServoController.get(1).getServoPosition(0); // Flapper Right
                ServoController.get(1).getServoPosition(1); // Flapper Left
                ServoController.get(1).getServoPosition(2); // Hood Right
                ServoController.get(1).getServoPosition(3); // Hood Left
            }

            /** @noinspection DataFlowIssue*/
            @Override
            public void write()
            {
                //Assuming that 0 is the Control Hub and 1 is the Expansion Hub (NOT VERIFIED)
                DcMotorController.get(0).setMotorPower(0, bulkValues.DcMotorValues.get(0));
                DcMotorController.get(0).setMotorPower(1, bulkValues.DcMotorValues.get(1));
                DcMotorController.get(0).setMotorPower(2, bulkValues.DcMotorValues.get(2));
                DcMotorController.get(0).setMotorPower(3, bulkValues.DcMotorValues.get(3));
                DcMotorController.get(1).setMotorPower(0, bulkValues.DcMotorValues.get(4));
                DcMotorController.get(1).setMotorPower(1, bulkValues.DcMotorValues.get(5)*calculateNormalizedVoltage(10.0));
                DcMotorController.get(1).setMotorPower(2, bulkValues.DcMotorValues.get(6)*calculateNormalizedVoltage(10.0));
                DcMotorController.get(1).setMotorPower(3, bulkValues.DcMotorValues.get(7));

                ServoController.get(0).setServoPosition(0, bulkValues.ServoValues.get(0));
                ServoController.get(0).setServoPosition(1, bulkValues.ServoValues.get(1));
//                ServoController.get(0).setServoPosition(2, bulkValues.ServoValues.get(2));
//                ServoController.get(0).setServoPosition(3, bulkValues.ServoValues.get(3));
                ServoController.get(1).setServoPosition(0, bulkValues.ServoValues.get(4));
                ServoController.get(1).setServoPosition(1, bulkValues.ServoValues.get(5));
                ServoController.get(1).setServoPosition(2, bulkValues.ServoValues.get(6));
                ServoController.get(1).setServoPosition(3, bulkValues.ServoValues.get(7));
            }
        };

        this.flywheel=new Turret(mode, powerManager);
        this.storage = new Storage(mode, powerManager);
        this.intake = new Intake(mode, powerManager);
        this.drive = new MecanumDrive(mode.hardwareMap, powerManager, new Pose2d(0, 0, 0));
        this.turret = new Turret(mode, powerManager);
        this.cam = new Limelight(mode);
        this.color = color;
    }


    public Command reset()
    {
        return new SequentialCommand(
                new ParallelCommand(
                        turret.reset(),
                        storage.flapper.reset(),
                        storage.spindexer.reset()
                )
        );
    }

    public Command update()
    {
        return new ParallelCommand(
//                cam.getPose(color,drive.localizer),
                Command.builder().update(drive::updatePoseEstimate).build(),
                turret.update(),
                storage.update(),
                Command.builder().update(powerManager::write).build()
                );
    }
}

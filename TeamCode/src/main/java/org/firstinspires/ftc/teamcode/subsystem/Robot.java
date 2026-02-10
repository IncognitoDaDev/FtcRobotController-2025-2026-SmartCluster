package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxFtdiResetControlCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorEncoderPositionCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetServoPulseWidthCommand;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.ConditionalCommand;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.commands.ParallelCommand;
import com.smartcluster.oracleftc.commands.SequentialCommand;
import com.smartcluster.oracleftc.commands.WaitCommand;
import com.smartcluster.oracleftc.hardware.OracleLynxVoltageSensor;

import java.util.List;
import java.util.function.Supplier;

@Config
public class Robot {
    public static double nominalVoltage=10.0;
    private final OpMode opMode;
    private final boolean color;
    public final Turret flywheel;
    public final MecanumDrive drive;
    public final Intake intake;
    public final Storage storage;
    public final Turret turret;
    public final Limelight cam;

//    class BulkValues
//    {
//        double dexLeftServo = 0, dexRightServo = 0;
//        double flapperLeft = 0, flapperRight = 0;
//        double hoodLeft = 0, hoodRight = 0;
//
//        double turretMotorUp = 0, turretMotorDown = 0;
//        double turretMotorRot = 0;
//
//        double frontLeftMotor = 0, frontRightMotor = 0;
//        double backLeftMotor = 0, backRightMotor = 0;
//    }

    List<LynxModule> lynxModules;

    public Robot(OpMode mode,boolean color)
    {
//        BulkValues bulkValues = new BulkValues();

        this.opMode = mode;
        OracleLynxVoltageSensor voltageSensor = mode.hardwareMap.getAll(OracleLynxVoltageSensor.class).iterator().next();
        voltageSensor.setPolicy(OracleLynxVoltageSensor.OracleLynxVoltageSensorPolicy.CACHED);
        voltageSensor.setVoltageCacheFreshness(50);
        this.flywheel=new Turret(mode);
        this.storage = new Storage(mode);
        this.intake = new Intake(mode);
        this.drive = new MecanumDrive(mode.hardwareMap, opMode.telemetry);
        this.turret = new Turret(mode);
        this.cam = new Limelight(mode);
        this.color = color;

        lynxModules = opMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule lynxModule : lynxModules)
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    public void read()
    {
        for (LynxModule lynxModule : lynxModules) {
            lynxModule.clearBulkCache();
            lynxModule.getBulkData();
        }
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
                drive.update(),
//                new InstantCommand(drive::updatePoseEstimate),
                turret.update(),
                storage.update()
        );
    }

    public Command slotRewind(Supplier<Boolean> condition){
        return new ConditionalCommand(condition, intake.outake(), new WaitCommand(1));
    }

}

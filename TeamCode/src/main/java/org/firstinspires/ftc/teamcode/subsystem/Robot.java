package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.commands.ParallelCommand;
import com.smartcluster.oracleftc.commands.SequentialCommand;
import com.smartcluster.oracleftc.hardware.OracleLynxVoltageSensor;

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

    public Robot(OpMode mode,boolean color)
    {

        this.opMode = mode;
        OracleLynxVoltageSensor voltageSensor = mode.hardwareMap.getAll(OracleLynxVoltageSensor.class).iterator().next();
        voltageSensor.setPolicy(OracleLynxVoltageSensor.OracleLynxVoltageSensorPolicy.CACHED);
        voltageSensor.setVoltageCacheFreshness(50);
        this.flywheel=new Turret(mode);
        this.storage = new Storage(mode);
        this.intake = new Intake(mode);
        this.drive = new MecanumDrive(mode.hardwareMap, new Pose2d(0, 0, 0));
        this.turret = new Turret(mode);
        this.cam = new Limelight(mode);
        this.color = color;


    }

    public Command reset()
    {
        return new SequentialCommand(
                new ParallelCommand(
                        turret.reset(),
                        storage.flapper.reset()
                ),
                storage.spindexer.reset()
        );
    }

    public Command update()
    {
        return new ParallelCommand(
                cam.getPose(color,drive.localizer),
//                Command.builder().update(drive::updatePoseEstimate).build(),
                turret.update(),
                storage.update()
        );
    }

}

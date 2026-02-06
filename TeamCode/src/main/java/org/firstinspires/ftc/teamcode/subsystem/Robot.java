package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.ConditionalCommand;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.commands.ParallelCommand;
import com.smartcluster.oracleftc.commands.SequentialCommand;
import com.smartcluster.oracleftc.commands.WaitCommand;
import com.smartcluster.oracleftc.hardware.OracleLynxVoltageSensor;

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

    public Robot(OpMode mode,boolean color)
    {

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
                turret.update(),
                storage.update()
        );
    }

    public Command slotRewind(Supplier<Boolean> condition){
        return new ConditionalCommand(condition,intake.outake(),new WaitCommand(1));
    }

}

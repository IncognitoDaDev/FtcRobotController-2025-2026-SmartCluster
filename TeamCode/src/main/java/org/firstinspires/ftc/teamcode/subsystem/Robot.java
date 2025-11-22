package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.ParallelCommand;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;

public class Robot{
    public final Turret turret;
    public final MecanumDrive drive;
    private OpMode opmode;
    private HardwareMap hardwareMap;

    public Robot(OpMode opMode) {
        this.opmode= opMode;
        this.drive = new MecanumDrive(opMode,opMode.hardwareMap);
        this.turret = new Turret(opMode,"turret");


    }
    public Command update()
    {
        return new ParallelCommand(
                turret.hood.update(),
                turret.turret.update(),
                turret.rotation.update()
        );
    }

    public Command reset(){
        return new ParallelCommand(
                turret.hood.reset(),
                turret.turret.reset(),
                turret.rotation.reset()
        );


    }

}

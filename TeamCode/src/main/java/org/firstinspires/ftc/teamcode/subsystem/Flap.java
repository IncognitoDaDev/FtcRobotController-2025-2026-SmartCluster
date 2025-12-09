package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.hardware.subsystem.ServoActuator;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;
import com.smartcluster.oracleftc.math.control.TrapezoidalMotionProfile;


public class Flap extends Subsystem {

    private ServoImplEx fservo1;
    private ServoImplEx fservo2;

    public ServoActuator flapper;


    public Flap(OpMode mode) {
        super(mode);
        fservo1=hardwareMap.get(ServoImplEx.class, "fservo1");
        fservo2=hardwareMap.get(ServoImplEx.class, "fservo2");
        fservo2.setDirection(Servo.Direction.REVERSE);

        flapper = new ServoActuator(this, "Flap", new TrapezoidalMotionProfile(0.48, 0.45, 1),fservo1)
        {

            @Override
            public Command reset() {
                return null;
            }

            @Override
            public boolean setTarget(double target) {
                return false;
            }
        };

    }

}

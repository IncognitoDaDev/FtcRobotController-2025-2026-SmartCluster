package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.hardware.subsystem.Actuator;
import com.smartcluster.oracleftc.hardware.subsystem.ServoActuator;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;
import com.smartcluster.oracleftc.math.DualNum;
import com.smartcluster.oracleftc.math.Time;
import com.smartcluster.oracleftc.math.control.PIDController;
import com.smartcluster.oracleftc.math.control.TrapezoidalMotionProfile;

public class Hood extends Subsystem {
    public ServoImplEx s1,s2;

    public static final double HOOD_MIN_ANGLE = 50; // Angle for shooting far
    public static final double HOOD_MAX_ANGLE = 60; // Angle for shooting close

    private ServoActuator hood;

    public Hood(OpMode opMode) {
        super(opMode);
        s1 = hardwareMap.get(ServoImplEx.class,"hoodServo1");
        s2 = hardwareMap.get(ServoImplEx.class,"hoodServo2");
        s2.setDirection(Servo.Direction.REVERSE);

        hood = new ServoActuator(this, "hood", new TrapezoidalMotionProfile(10,16,16),s1,s2) {
            @Override
            public boolean setTarget(double target) {
                if (target < HOOD_MIN_ANGLE) {
                    target = HOOD_MIN_ANGLE;
                }
                if (target > HOOD_MAX_ANGLE) {
                    target = HOOD_MAX_ANGLE;
                }


                this.target.set(target);


                return true;
            }


            @Override
            public Command reset() {
                return new InstantCommand(() -> {
                    setTarget(HOOD_MIN_ANGLE);
                });
            }
            public Command goToAngle(double angle) {
                return new InstantCommand(() -> {
                    hood.setTarget(angle);
                });
            }
        };

    }
}

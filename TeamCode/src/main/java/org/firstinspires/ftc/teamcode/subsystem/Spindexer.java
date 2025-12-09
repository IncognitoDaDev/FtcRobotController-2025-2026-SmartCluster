package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.hardware.subsystem.ServoActuator;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;

public class Spindexer extends Subsystem {
    public CRServoImplEx dexLeft,dexRight;
    public static final double degreePerposition = 1.0/360;
    public static final double NEXT_BALL = 120*degreePerposition; // Position for switching balls
    public static final double SHOOT_POSE = 60 * degreePerposition; // Position for switching mode
    ElapsedTime time = new ElapsedTime();
    public static double[] period = {300,150};

    public Spindexer(OpMode opMode) {
        super(opMode);
        dexRight = hardwareMap.get(CRServoImplEx.class,"dexRight");
        dexLeft = hardwareMap.get(CRServoImplEx.class,"dexLeft");


    }
    //x is for the direction of sorting
   public Command nextBall(int x){
        return  Command.builder()
                .init(() -> {
                    time.reset();
                    dexRight.setPower(x*0.8);
                    dexLeft.setPower(x*0.8);
                })
                .finished(() -> {
                    return time.milliseconds() > period[0];
                })
                .end((interrupted) -> {
                    // THIS IS THE GOAL: Reset the encoder to zero at the hard stop.
                    dexLeft.setPower(0);
                    dexRight.setPower(0);
                    // Set a small holding power or stop the motor
                })
                .build();
   }
    public Command switchPhase(){
        return  Command.builder()
                .init(() -> {
                    time.reset();
                    dexRight.setPower(0.8);
                    dexLeft.setPower(0.8);
                })
                .finished(() -> {
                    return time.milliseconds() > period[1];
                })
                .end((interrupted) -> {
                    // THIS IS THE GOAL: Reset the encoder to zero at the hard stop.
                    dexLeft.setPower(0);
                    dexRight.setPower(0);
                    // Set a small holding power or stop the motor
                })
                .build();
    }
}



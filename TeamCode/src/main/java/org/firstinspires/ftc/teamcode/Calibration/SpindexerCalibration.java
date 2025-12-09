package org.firstinspires.ftc.teamcode.Calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.WaitCommand;

@TeleOp
public class SpindexerCalibration extends LinearOpMode {
    public CRServoImplEx dexLeft,dexRight;
    public static final double degreePerposition = 1.0/360;
    ElapsedTime time = new ElapsedTime();
    public static double[] period = {300,150};
    @Override
    public void runOpMode() throws InterruptedException {
        dexRight = hardwareMap.get(CRServoImplEx.class,"dexRight");
        dexLeft = hardwareMap.get(CRServoImplEx.class,"dexLeft");
        CommandScheduler scheduler = new CommandScheduler();
        waitForStart();
        if(!opModeIsActive())return;
        while(opModeIsActive()){
            Command.run(nextBall(1));
            Command.run(new WaitCommand(10000));
            Command.run(switchPhase());

        }


    }
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

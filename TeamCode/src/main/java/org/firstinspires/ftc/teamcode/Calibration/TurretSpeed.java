package org.firstinspires.ftc.teamcode.Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.commands.ParallelCommand;
import com.smartcluster.oracleftc.fsm.FSM;
import com.smartcluster.oracleftc.utils.ProcessedGamepad;

import org.firstinspires.ftc.teamcode.opmode.DuoMode;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

import java.util.List;

@TeleOp
public class TurretSpeed extends LinearOpMode {
    Servo servoflap;

    @Override
    public void runOpMode() throws InterruptedException {
        servoflap = hardwareMap.get(Servo.class,"flapperLeft");
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.xWasPressed())servoflap.setPosition(1);
            else servoflap.setPosition(0);
        }
    }
}

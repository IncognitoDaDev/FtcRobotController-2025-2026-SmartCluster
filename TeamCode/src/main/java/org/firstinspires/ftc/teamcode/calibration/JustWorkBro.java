package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorEncoderPositionCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetServoPulseWidthCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import java.util.List;
@TeleOp
public class JustWorkBro extends LinearOpMode {


    List<LynxModule> lynxModuleList;

    @Override
    public void runOpMode() throws InterruptedException {

        List<LynxModule> lynxModuleList = hardwareMap.getAll(LynxModule.class);
        DcMotorImplEx hey = hardwareMap.get(DcMotorImplEx.class, "turretUp");
        List<LynxDcMotorController> DcMotorController;
        List<LynxServoController> ServoController;

        waitForStart();

        DcMotorController = hardwareMap.getAll(LynxDcMotorController.class);
        ServoController = hardwareMap.getAll(LynxServoController.class);

        while(opModeIsActive())
        {
//            for (LynxDcMotorController controller :)
//            DcMotorController.get(0).initializeHardware();

            telemetry.addData("Vel", hey.getVelocity());
            telemetry.update();
        }
    }
}

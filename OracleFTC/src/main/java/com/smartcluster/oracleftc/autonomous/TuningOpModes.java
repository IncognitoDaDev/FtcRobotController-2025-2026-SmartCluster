package com.smartcluster.oracleftc.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.smartcluster.oracleftc.utils.ProcessedGamepad;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

import java.util.function.Function;

public class TuningOpModes {

    public static final String GROUP = "tuning";

    private static OpModeMeta metaForClass(Class<? extends OpMode> opmode) {
        return new OpModeMeta.Builder()
                .setName(opmode.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .setSource(OpModeMeta.Source.EXTERNAL_LIBRARY)
                .build();
    }

    private abstract static class TuningOpMode extends LinearOpMode
    {
        protected Function<OpMode, MecanumDrive> driveProducer;
        public TuningOpMode(Function<OpMode, MecanumDrive> driveProducer)
        {
            this.driveProducer=driveProducer;
        }
    }

    public static class MecanumDirectionDebugger extends TuningOpMode {
        public MecanumDirectionDebugger(Function<OpMode, MecanumDrive> driveProducer) {
            super(driveProducer);
        }

        @Override
        public void runOpMode() throws InterruptedException {
            MecanumDrive drive=driveProducer.apply(this);
            waitForStart();
            ProcessedGamepad gamepad = new ProcessedGamepad(gamepad1);
            while(opModeIsActive()&&!isStopRequested())
            {
                double[] motorPowers = new double[4];
                if(gamepad.right_trigger.get()>0.1) motorPowers[0]=0.5;
                if(gamepad.right_bumper.get()) motorPowers[1]=0.5;
                if(gamepad.right_trigger.get()>0.1) motorPowers[2]=0.5;
                if(gamepad.left_bumper.get()) motorPowers[3]=0.5;
                drive.setMotorPowers(motorPowers);
                gamepad.process();
            }
        }
    }

    public static class LocalizationTuner extends TuningOpMode {

        public LocalizationTuner(Function<OpMode, MecanumDrive> driveProducer) {
            super(driveProducer);
        }

        @Override
        public void runOpMode() throws InterruptedException {
            MecanumDrive drive = driveProducer.apply(this);
//            drive.localizer.tune(this);
        }
    }



    public static void register(OpModeManager manager, Function<OpMode, MecanumDrive> driveProducer)
    {
        manager.register(metaForClass(MecanumDirectionDebugger.class), new MecanumDirectionDebugger(driveProducer));
        manager.register(metaForClass(LocalizationTuner.class), new LocalizationTuner(driveProducer));
    }
}

package org.firstinspires.ftc.teamcode.subsystem;

import static com.sun.tools.javac.jvm.ByteCodes.swap;

import static java.util.Collections.swap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class CameraRecognition extends Subsystem {
    private final Limelight3A camera;
    private String[] order ={"Empty","PURPLE","PURPLE","PURPLE"};

    public CameraRecognition(OpMode opMode) {
        super(opMode);
        camera = hardwareMap.get(Limelight3A.class, "Turret Webcam");
        telemetry.setMsTransmissionInterval(11);


    }


    public void Start(Limelight3A limelight) {
        camera.pipelineSwitch(2);
        camera.start();

    }

    public void getResults(Limelight3A limelight,int pipelineNr) {
        camera.pipelineSwitch(pipelineNr);
        LLResult result = limelight.getLatestResult();

        if (result != null) {
            if (result.isValid()) {
                switch(pipelineNr) {
                    case 2:
                        int rezultat = result.getBotposeTagCount();
                        telemetry.addData("Botpose Tag Count", result.getBotposeTagCount());
                        switch(rezultat) {
                            case 21:

                        }
                    case 3:

                }
            }
        }
    }
}

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
    private String[] order ={"Empty","PURPLE","GREEN","PURPLE"};

    public CameraRecognition(OpMode opMode) {
        super(opMode);
        camera = hardwareMap.get(Limelight3A.class, "Turret Webcam");
        telemetry.setMsTransmissionInterval(11);


    }


    public void Start(Limelight3A limelight) {
        camera.start();

    }
    public void ColorOrder(Limelight3A ll){
        ll.pipelineSwitch(2);
    }

    public void getResults(Limelight3A limelight,int pipelineNr) {
        camera.pipelineSwitch(pipelineNr);
        LLResult result = limelight.getLatestResult();

        if (result != null) {
            if (result.isValid()) {
                switch(pipelineNr) {
                    case 2:
                        int rezultat = result.getBotposeTagCount();

                        switch(rezultat) {
                            case 21:
                                setOrder(new String[]{"Empty","GREEN","PURPLE","PURPLE"});
                                telemetry.addData("Order",order);
                                break;
                            case 22:
                                setOrder(new String[]{"Empty","PURPLE","GREEN","PURPLE"});
                                telemetry.addData("Order",order);
                                break;
                            case 23:
                                setOrder(new String[]{"Empty","PURPLE","PURPLE","GREEN"});
                                telemetry.addData("Order",order);
                                break;
                        }
                    case 3:




                }
            }
        }
    }

    public String[] getOrder() {
        return order;
    }

    public void setOrder(String[] order) {
        this.order = order;
    }
}

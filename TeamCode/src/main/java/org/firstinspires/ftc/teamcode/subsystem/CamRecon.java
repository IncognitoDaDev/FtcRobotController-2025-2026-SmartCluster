package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;

import java.util.List;

import org.firstinspires.ftc.teamcode.subsystem.Storage.ArtifactColor;

public class CamRecon extends Subsystem {

    private final Limelight3A limelight;
    private boolean enabled = true;
    private ArtifactColor[] lockedPattern = new ArtifactColor[]{ArtifactColor.GREEN, ArtifactColor.GREEN, ArtifactColor.GREEN};

    public CamRecon(OpMode opMode) {
        super(opMode);
        limelight = hardwareMap.get(Limelight3A.class, "Webcam Turret");
        limelight.start();
    }

    public ArtifactColor[] getAprilTagPattern() {
        if (!enabled) return lockedPattern;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return lockedPattern;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) return lockedPattern;

        for (LLResultTypes.FiducialResult tag : fiducials) {
            switch (tag.getFiducialId()) {
                case 21:
                    lock(new ArtifactColor[]{ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE});
                    break;
                case 22:
                    lock(new ArtifactColor[]{ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE});
                    break;
                case 23:
                    lock(new ArtifactColor[]{ArtifactColor.PURPLE, ArtifactColor.PURPLE, ArtifactColor.GREEN});
                    break;
            }
        }
        return lockedPattern;
    }

    private void lock(ArtifactColor[] pattern) {
        lockedPattern = pattern;
        enabled = false;
        limelight.stop();
    }

    public double checker() {
        LLStatus status = limelight.getStatus();
        return status.getPipelineIndex();
    }

    public void reset() {
        enabled = true;
        lockedPattern = new ArtifactColor[]{ArtifactColor.GREEN, ArtifactColor.GREEN, ArtifactColor.GREEN};
        limelight.start();
    }
}



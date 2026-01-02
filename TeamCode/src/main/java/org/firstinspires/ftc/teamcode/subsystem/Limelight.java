package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;

import java.util.List;

import org.firstinspires.ftc.teamcode.subsystem.Storage.ArtifactColor;

public class Limelight extends Subsystem {

    private final Limelight3A limelight;
    private ArtifactColor[] order = {ArtifactColor.EMPTY, ArtifactColor.EMPTY, ArtifactColor.EMPTY};
    public Limelight(OpMode opMode) {
        super(opMode);
        limelight = hardwareMap.get(Limelight3A.class, "Webcam Turret");
    }

    boolean isFinished = false;

    public ArtifactColor[] getOrder()
    {
        return order;
    }
    public Command scanOrder()
    {
        ElapsedTime timer = new ElapsedTime();
        return Command.builder()
                .init(() ->
                {
                    isFinished = false;
                    timer.reset();
                    limelight.start();
                })
                .update(() ->
                {
                    LLResult result = limelight.getLatestResult();
                    if (result != null && result.isValid())
                    {
                        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                        if (!fiducials.isEmpty())
                        {
                            for (LLResultTypes.FiducialResult tag : fiducials) {
                                switch (tag.getFiducialId()) {
                                    case 21:
                                        order = new ArtifactColor[]{ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE};
                                        limelight.stop();
                                        isFinished = true;
                                        break;
                                    case 22:
                                        order = new ArtifactColor[]{ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE};
                                        limelight.stop();
                                        isFinished = true;
                                        break;
                                    case 23:
                                        order = new ArtifactColor[]{ArtifactColor.PURPLE, ArtifactColor.PURPLE, ArtifactColor.GREEN};
                                        limelight.stop();
                                        isFinished = true;
                                        break;
                                }
                            }
                        }
                    }
                })
                .finished(() -> timer.milliseconds() > 250 || isFinished)
                .build();
    }

    public void reset() {
       limelight.start();
    }
}
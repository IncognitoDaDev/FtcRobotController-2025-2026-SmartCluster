package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class ColorType {

    ColorType(Storage.ArtifactColor TypeObj, double[] transparency, double[] red, double[] green, double[] blue)
    {
        this.TypeObj = TypeObj;

    }

    Storage.ArtifactColor TypeObj;
    public double[] TRANSPARENCY_THRESHOLD;
    public double[] RED_THRESHOLD;
    public double[] GREEN_THRESHOLD;
    public double[] BLUE_THRESHOLD;

    // RGB
    public static ColorType Purple = new ColorType(Storage.ArtifactColor.PURPLE,
            new double[] {0, 255}, new double[]{0, 255}, new double[]{0, 255}, new double[]{0, 255});

    public static ColorType Green = new ColorType(Storage.ArtifactColor.GREEN,
            new double[] {0, 255}, new double[]{0, 255}, new double[]{0, 255}, new double[]{0, 255});

//    public static ColorType Nothing = new ColorType(Storage.ArtifactColor.EMPTY,
//            new double[] {0, 255}, new double[]{0, 255}, new double[]{0, 255}, new double[]{0, 255});

    public static int identifyObj(RevColorSensorV3 sensor)
    {
        int data = sensor.argb();
        return data;
    }
}

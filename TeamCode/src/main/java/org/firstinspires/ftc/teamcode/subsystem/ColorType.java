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
            new double[] {0, 255}, new double[]{0, 6}, new double[]{0, 8}, new double[]{9, 255});

    public static ColorType Green = new ColorType(Storage.ArtifactColor.GREEN,
            new double[] {0, 255}, new double[]{0, 5}, new double[]{8, 255}, new double[]{0, 10});

//    public static ColorType Wall = new ColorType(Storage.ArtifactColor.WALL,
//            new double[] {120, 140}, new double[]{0, 255}, new double[]{8, 255}, new double[]{0, 10});

//    public static ColorType Nothing = new ColorType(Storage.ArtifactColor.EMPTY,
//            new double[] {0, 255}, new double[]{0, 255}, new double[]{0, 255}, new double[]{0, 255});

    public Storage.ArtifactColor identifyObj(RevColorSensorV3 sensor)
    {
        NormalizedRGBA data = sensor.getNormalizedColors();

        if (data.alpha*256 > 240) { //Something exists... and it's a ball
            if (data.green * 256 > 8.5) // Checking if is GREEN
                return Storage.ArtifactColor.GREEN;
            //if (data.blue * 256 > 9) // Checking if its
            else
                return (Storage.ArtifactColor.PURPLE); // Must be purple then
        }

        return Storage.ArtifactColor.EMPTY;
    }
}

package org.firstinspires.ftc.teamcode.subsystem;

import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;

public class Color
{
    // DOUBLES HAVE A LEFT VALUE AND A RIGHT VALUE, REPRESENTING THE RANGES

    public static class Purple
    {
        final static public double[] RED_THRESHOLD = {30 , 35};
        final public static double[] GREEN_THRESHOLD = {50 , 58};
        final public static double[] BLUE_THRESHOLD = {70, 80};
        final public static double[] TRANSPARENCY_THRESHOLD = {0.0, 255};
    }

    public static class Green
    {
        final static public  double[] RED_THRESHOLD = {70, 77};
        final static public double[] GREEN_THRESHOLD = {80, 87};
        final static public double[] BLUE_THRESHOLD = {68, 75};
        final static public double[] TRANSPARENCY_THRESHOLD = {0.0, 255};
    }

    public static class Nimic
    {
        final static public double[] RED_THRESHOLD = { 23 , 30 };
        final static public double[] GREEN_THRESHOLD= { 78,86 } ;
        final static public double[] BLUE_THRESHOLD= { 40,47 };
        final static public double[] TRANSPARENCY_THRESHOLD= { 0,255 };


}
}


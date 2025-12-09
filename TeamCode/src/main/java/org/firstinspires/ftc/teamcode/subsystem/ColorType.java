package org.firstinspires.ftc.teamcode.subsystem;

public class ColorType {

    public class V3
    {
        public double[] RED_THRESHOLD;
        public double[] GREEN_THRESHOLD;
        public double[] BLUE_THRESHOLD;
        //static public double[] TRANSPARENCY_THRESHOLD;

        V3(double[] R, double[] G, double[] B)
        {
            this.RED_THRESHOLD = R;
            this.GREEN_THRESHOLD = G;
            this.BLUE_THRESHOLD = B;
        }
    }

    public class V2
    {
        public double[] RED_THRESHOLD ;
        public double[] GREEN_THRESHOLD;
        public double[] BLUE_THRESHOLD;
        //static public double[] TRANSPARENCY_THRESHOLD;

        V2(double[] R, double[] G, double[] B)
        {
            this.RED_THRESHOLD = R;
            this.GREEN_THRESHOLD = G;
            this.BLUE_THRESHOLD = B;
        }
    }

    public enum IdentityObject
    {
        PURPLE,
        GREEN,
        WALL,
        EMPTY
    }
    public V3 v3;
    public V2 v2;
    public IdentityObject identity;

    public static ColorType Purple = new ColorType(IdentityObject.PURPLE, new double[]{30, 35}, new double[]{50 , 58}, new double[]{70, 80},
            new double[]{30, 35}, new double[]{50 , 58}, new double[]{70, 80});
    public static ColorType Green = new ColorType(IdentityObject.GREEN, new double[]{70, 77}, new double[]{80, 87}, new double[]{68, 75},
            new double[]{30, 35}, new double[]{50 , 58}, new double[]{70, 80});
    public static ColorType Wall = new ColorType(IdentityObject.WALL, new double[]{30, 35}, new double[]{50 , 58}, new double[]{70, 80},
            new double[]{30, 35}, new double[]{50 , 58}, new double[]{70, 80});
    public static ColorType Nothing = new ColorType(IdentityObject.EMPTY, new double[]{30, 35}, new double[]{50 , 58}, new double[]{70, 80},
            new double[]{30, 35}, new double[]{50 , 58}, new double[]{70, 80});

    ColorType(ColorType.IdentityObject identity, double[] REDv3, double[] GREENv3, double[] BLUEv3, double[] REDv2, double[] GREENv2, double[] BLUEv2)
    {
        this.identity = identity;

        this.v3 = new V3(REDv3, GREENv3, BLUEv3);
        this.v2 = new V2(REDv2, GREENv2, BLUEv2);
    }
}

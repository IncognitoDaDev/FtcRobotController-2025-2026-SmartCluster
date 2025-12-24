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

    // RGB
    public static ColorType Purple = new ColorType(IdentityObject.PURPLE,
            new double[]{300, 1800}, new double[]{1200, 2200}, new double[]{2200, 3300},
            new double[]{300, 600}, new double[]{320, 500}, new double[]{400, 850});
    public static ColorType Green = new ColorType(IdentityObject.GREEN,
            new double[]{300, 1000}, new double[]{2200, 3400}, new double[]{1500, 2600},
            new double[]{150, 450}, new double[]{500, 900}, new double[]{200, 650});
    public static ColorType Wall = new ColorType(IdentityObject.WALL,
            new double[]{100, 140}, new double[]{130, 500}, new double[]{190, 550},
            new double[]{10, 60}, new double[]{30 , 90}, new double[]{30, 90});
    public static ColorType Nothing = new ColorType(IdentityObject.EMPTY,
            new double[]{0, 50}, new double[]{0 , 80}, new double[]{0, 70},
            new double[]{0, 30}, new double[]{0, 30}, new double[]{0, 30});

    ColorType(ColorType.IdentityObject identity, double[] REDv3, double[] GREENv3, double[] BLUEv3, double[] REDv2, double[] GREENv2, double[] BLUEv2)
    {
        this.identity = identity;

        this.v3 = new V3(REDv3, GREENv3, BLUEv3);
        this.v2 = new V2(REDv2, GREENv2, BLUEv2);
    }
}

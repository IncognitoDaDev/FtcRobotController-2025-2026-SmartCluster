package com.smartcluster.oracleftc;

import org.junit.Test;
import org.w3c.dom.Element;

import static org.junit.Assert.*;

import com.smartcluster.oracleftc.math.DualNum;
import com.smartcluster.oracleftc.math.Time;
import com.smartcluster.oracleftc.math.control.TrapezoidalMotionProfile;

/**
 * Example local unit test, which will execute on the development machine (host).
 *
 * @see <a href="http://d.android.com/tools/testing">Testing documentation</a>
 */
public class TrapezoidalMotionProfileTest {
    @Test
    public void addition_isCorrect()
    {
        long distance = 1000;
        TrapezoidalMotionProfile motionProfile = new TrapezoidalMotionProfile(3000,20000,10000);



        DualNum<Time> mp;
        double time = 0.1;
        do
        {

            mp = motionProfile.getMotionState(distance, time);
            System.out.print(time);
            System.out.print(" ");
            System.out.print(mp.get(0));
            System.out.print(" ");
            System.out.println(mp.get(1));
            time+=0.05;
        }while(mp.get(0)<distance);


    }
}
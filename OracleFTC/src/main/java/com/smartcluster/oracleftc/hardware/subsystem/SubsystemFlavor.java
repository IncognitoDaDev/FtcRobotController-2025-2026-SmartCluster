package com.smartcluster.oracleftc.hardware.subsystem;

public enum SubsystemFlavor {
    ControlHubOnly,
    ExpansionHubOnly,
    Mixed;
    public static SubsystemFlavor merge(SubsystemFlavor a, SubsystemFlavor b)
    {
        if(a==null)
            return b;
        else if(b==null)
            return a;
        else if(a!=b)
                return Mixed;
        else return a;
    }
}

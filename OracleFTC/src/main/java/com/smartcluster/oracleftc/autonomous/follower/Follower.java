package com.smartcluster.oracleftc.autonomous.follower;

import com.smartcluster.oracleftc.autonomous.MecanumDrive;

public abstract class Follower {
    protected MecanumDrive drive;
    public Follower(MecanumDrive drive)
    {
        this.drive=drive;
    }
}

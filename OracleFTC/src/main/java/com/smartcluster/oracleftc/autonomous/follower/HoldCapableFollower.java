package com.smartcluster.oracleftc.autonomous.follower;

import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.math.Pose2d;
import java.util.function.Supplier;

public interface HoldCapableFollower {
    Command hold(Pose2d pose);
}

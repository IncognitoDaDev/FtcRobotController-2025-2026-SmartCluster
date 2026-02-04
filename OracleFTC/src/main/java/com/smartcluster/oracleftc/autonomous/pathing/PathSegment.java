package com.smartcluster.oracleftc.autonomous.pathing;

import com.smartcluster.oracleftc.math.Pose2d;
import com.smartcluster.oracleftc.math.Rotation2d;
import com.smartcluster.oracleftc.math.Vector2d;

import java.util.function.Function;

public class PathSegment {

    public final BezierCurve curve;
    private final Function<Double, Rotation2d> heading;


    public PathSegment(BezierCurve curve, Function<Double, Rotation2d> heading) {
        this.curve = curve;
        this.heading = heading;
    }


}

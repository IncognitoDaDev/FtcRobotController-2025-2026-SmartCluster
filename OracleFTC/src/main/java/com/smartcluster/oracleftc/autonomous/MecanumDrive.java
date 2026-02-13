//package com.smartcluster.oracleftc.autonomous;
//
//
//
//import com.qualcomm.hardware.lynx.LynxVoltageSensor;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.smartcluster.oracleftc.commands.Command;
//import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;
//import com.smartcluster.oracleftc.math.Pose2dDual;
//import com.smartcluster.oracleftc.math.Time;
//
//public abstract class MecanumDrive extends Subsystem {
//
//    protected DcMotorEx frontRightMotor, backRightMotor, frontLeftMotor, backLeftMotor;
//    protected LynxVoltageSensor voltageSensor;
//    protected Localizer localizer;
//
//    public MecanumDrive(OpMode opMode) {
//        super(opMode);
//    }
//
//    public abstract void drawRobot(Pose2dDual<Time> pose);
//
//    public final Command update()
//    {
//        return Command.builder()
//                .update(()->{
//                    localizer.update();
//                    drawRobot(localizer.getPose());
//
//                })
//                .requires(this)
//                .build();
//    }
//
////    public final Command follow(PathSegment... pathSegments)
////    {
////        AtomicInteger pathSegmentIndex= new AtomicInteger();
////        return Command.builder()
////                .init(()->{
////                    pathSegmentIndex.set(0);
////                })
////                .update(()->{
////                    Pose2dDual<Time> currentPose = getPose();
////                    PathSegment currentPathSegment = pathSegments[pathSegmentIndex.get()];
////                    double closestPointT = pathSegments[pathSegmentIndex.get()].curve.getClosestT(currentPose.position.value());
////
////                    Vector2d translationalVector = currentPathSegment.curve.getPoint(closestPointT).minus(currentPose.value().position);
////                    Vector2d correctionVector = mass * centripetalForceScaling * currentPathSegment.curve.getCurvature(closestPointT) * Math.pow(currentPose.velocity().value().linearVel.dot(currentPathSegment.curve.getDerivative(closestPointT).normalized()), 2) *currentPathSegment.curve.getCurvature(closestPointT);
////
////
////                })
////                .finished(()->{
////                    if(pathSegmentIndex.get()!=pathSegments.length) return false;
////                })
////                .end((interrupted)->{
////
////                })
////                .requires(this)
////                .build();
////    }
////
////    public final Command hold(Pose2d pose)
////    {
////
////    }
//
//
//
//
//    public final void setPose(Pose2dDual<Time> pose)
//    {
//        localizer.setPose(pose);
//    }
//
//    public final Pose2dDual<Time> getPose() { return localizer.getPose(); }
//}

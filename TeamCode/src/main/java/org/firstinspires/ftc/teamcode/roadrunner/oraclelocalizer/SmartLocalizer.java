package org.firstinspires.ftc.teamcode.roadrunner.oraclelocalizer;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.ftcrobotcontroller.BuildConfig;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.smartcluster.oracleftc.hardware.OracleGoBildaPinpoint;
import com.smartcluster.oracleftc.math.DualNum;
import com.smartcluster.oracleftc.math.Pose2dDual;
import com.smartcluster.oracleftc.math.Rotation2d;
import com.smartcluster.oracleftc.math.Rotation2dDual;
import com.smartcluster.oracleftc.math.Time;
import com.smartcluster.oracleftc.math.Twist2dDual;
import com.smartcluster.oracleftc.math.Vector2dDual;
import com.smartcluster.oracleftc.math.filters.LowPassFilter;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.LinkedList;
import java.util.Queue;

@Config
public class SmartLocalizer extends Localizer {

    // Bāraka Allāhu fī ChatGPT
    public class IMURotationTracker {
        private  class YawReading {
            double yawRadians;
            double timestamp;

            YawReading(double yawRadians, double timestamp) {
                this.yawRadians = yawRadians;
                this.timestamp = timestamp;
            }
        }

        private final Queue<YawReading> buffer = new LinkedList<>();
        private static final double WINDOW_SIZE = 0.05; // 50 ms window in seconds

        public double calculateAngularVelocity(double currentYawDegrees, double currentTime) {
            // Convert yaw from degrees to radians
            double currentYaw = currentYawDegrees;

            // Update buffer with new reading
            buffer.add(new YawReading(currentYaw, currentTime));

            // Remove outdated values (older than 50ms)
            while (buffer.size() > 1 && (currentTime - buffer.peek().timestamp) > WINDOW_SIZE) {
                buffer.poll();
            }

            // Compute angular velocity over the stored window
            if (buffer.size() > 1) {
                YawReading oldest = buffer.peek();
                double totalDeltaYaw = wrapAngle(currentYaw - oldest.yawRadians);
                double deltaT = currentTime - oldest.timestamp;
                if (deltaT > 0) {
                    return totalDeltaYaw / deltaT; // Angular velocity in radians per second
                }
            }

            return 0.0; // Default when buffer is not full
        }

        private double wrapAngle(double angle) {
            // Ensures yaw difference is within [-π, π] range to handle wrap-around correctly
            if (angle > Math.PI) {
                return angle - 2 * Math.PI;
            } else if (angle < -Math.PI) {
                return angle + 2 * Math.PI;
            }
            return angle;
        }
    }

    public static double parallelOffset=-4001.5408638757076*1/19.89436789;
    public static double perpendicularOffset=864.5412905604903*1/19.89436789;
    public static double mmPerTick=1/19.89436789;
    public static long pinpointTimeDelta = 1000;
    public static long pinpointRejectionThreshold = 4;
    private final AnalogInput canandgyro;
    private double gyroVoltageOffset;
    private final OracleGoBildaPinpoint pinpoint;
    public final com.acmerobotics.roadrunner.ftc.Encoder parallelEncoder, perpendicularEncoder;
    private final LowPassFilter headingVelFilter= new LowPassFilter(0.35);
    private final Telemetry telemetry;
    public SmartLocalizer(HardwareMap hardwareMap, Telemetry telemetry)
    {
        super(hardwareMap, telemetry);
        this.telemetry=telemetry;
        canandgyro = hardwareMap.get(AnalogInput.class, "canandgyro");
        pinpoint = hardwareMap.get(OracleGoBildaPinpoint.class, "pinpoint");
        parallelEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "backLeft")));
//        parallelEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        lastParallel = new DualNum<>(parallelEncoder.getPositionAndVelocity().position);

        perpendicularEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "frontLeft")));
//        perpendicularEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        lastPerpendicular = new DualNum<>(perpendicularEncoder.getPositionAndVelocity().position);
        gyroVoltageOffset = canandgyro.getVoltage();

        pinpoint.setEncoderResolution(19.89436789);
        pinpoint.setEncoderDirections(OracleGoBildaPinpoint.EncoderDirection.REVERSED, OracleGoBildaPinpoint.EncoderDirection.FORWARD);
        pinpoint.setOffsets(-parallelOffset, perpendicularOffset);

        pinpoint.resetPosAndIMU();
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            RobotLog.logStackTrace(e);
        }
    }

    private DualNum<Time> lastParallel, lastPerpendicular;
    private Rotation2dDual<Time> lastHeading = Rotation2dDual.constant(Rotation2d.exp(0),1);
    private final ElapsedTime deltaTime = new ElapsedTime();
    private final ElapsedTime pinpointTime = new ElapsedTime();
    private final IMURotationTracker tracker = new IMURotationTracker();
    @Override
    public final Twist2dDual<Time> update() {
//
//        if(BuildConfig.DEBUG)
//        {
//            telemetry.addData("rawGyroAngle", AngleUnit.normalizeDegrees((-canandgyro.getVoltage()) * 360.0 / 3.3));
//            telemetry.addData("offsetGyroAngle", AngleUnit.normalizeDegrees((canandgyro.getVoltage()-gyroVoltageOffset) * 360.0 / 3.3));
//            telemetry.addData("pinpointFrequency", pinpoint.getFrequency());
//
//            telemetry.addData("parallelEncoder", parallelEncoder.getPositionAndVelocity().position);
//            telemetry.addData("perpendicularEncoder", perpendicularEncoder.getPositionAndVelocity().position);
//        }

        double canandgyroHeading = Math.toRadians(AngleUnit.normalizeDegrees((canandgyro.getVoltage()-gyroVoltageOffset) * 360.0 / 3.3));

        DualNum<Time> parallel = new DualNum<>(parallelEncoder.getPositionAndVelocity().position, parallelEncoder.getPositionAndVelocity().velocity);
        DualNum<Time> perpendicular = new DualNum<>(perpendicularEncoder.getPositionAndVelocity().position, perpendicularEncoder.getPositionAndVelocity().velocity);
        Rotation2d heading = Rotation2d.exp(canandgyroHeading);

        DualNum<Time> parallelDelta = parallel.minus(lastParallel);
        DualNum<Time> perpendicularDelta = perpendicular.minus(lastPerpendicular);
        double headingDifference = heading.minus(lastHeading.value());
        double headingVel = headingVelFilter.update(tracker.calculateAngularVelocity(canandgyroHeading, System.nanoTime()/1E9));
        Rotation2dDual<Time> headingDelta=Rotation2dDual.exp(new DualNum<>(headingDifference, headingVel));
        deltaTime.reset();

        Twist2dDual<Time> updateTwist = new Twist2dDual<>(
                new Vector2dDual<>(
                    parallelDelta.minus(headingDelta.log().times(parallelOffset*(1/mmPerTick))).times(mmPerTick),
                    perpendicularDelta.minus(headingDelta.log().times(perpendicularOffset*(1/mmPerTick))).times(mmPerTick)
                ).div(25.4),
                headingDelta.log()
        );
        if(pinpointTime.milliseconds()>pinpointTimeDelta)
        {
            pinpoint.update();
            Pose2dDual<Time> newPose = pinpoint.getPose();

            if(((Double.isNaN(newPose.heading.log().get(0)) || Double.isNaN(newPose.position.x.get(0)) || Double.isNaN(newPose.position.y.get(0)))) ||
                pose.position.minus(newPose.position).sqrNorm().get(0) > pinpointRejectionThreshold*pinpointRejectionThreshold)
            {

            }else {
                pose=newPose;
            }
            pinpointTime.reset();
        }
        pose = new Pose2dDual<>(pose.value().plus(updateTwist.value()), pose.value().plus(updateTwist.value()).times(updateTwist.velocity()));
//        telemetry.addData("internalHeading", pose.heading.value().log());

        lastHeading=Rotation2dDual.constant(heading,1);
        lastParallel=new DualNum<>(parallel.get(0));
        lastPerpendicular=new DualNum<>(perpendicular.get(0));


        return updateTwist;
    }

    @Override
    public void setPose(Pose2dDual<Time> pose) {
        super.setPose(pose);
        gyroVoltageOffset=canandgyro.getVoltage()-pose.heading.log().get(0) *3.3/360;
        pinpoint.setPose(pose.value());
    }
}

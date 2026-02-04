package com.smartcluster.oracleftc.math.control;

import com.smartcluster.oracleftc.math.DualNum;
import com.smartcluster.oracleftc.math.Time;


public class TrapezoidalMotionProfile extends MotionProfile {
    public double maxVelocity, maxAcceleration, maxDeceleration;

    public TrapezoidalMotionProfile(double maxVelocity, double maxAcceleration, double maxDeceleration) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
    }

    public TrapezoidalMotionProfile(double maxVelocity, double maxAcceleration) {
        this(maxVelocity, maxAcceleration, maxAcceleration);
    }

    @Override
    public DualNum<Time> getMotionState(double distance, double time) {
        // Math to determine proper acceleration and deceleration periods
        double accelerationTime = Math.min(Math.sqrt((distance / (1 - maxAcceleration / -maxDeceleration)) / (maxAcceleration / 2)), maxVelocity / maxAcceleration);
        double decelerationTime = Math.min(Math.sqrt((distance / (1 - maxDeceleration / -maxAcceleration)) / (maxDeceleration / 2)), maxVelocity / maxDeceleration);

        double accelerationDistance = maxAcceleration / 2 * Math.pow(accelerationTime, 2);
        double decelerationDistance = maxDeceleration / 2 * Math.pow(decelerationTime, 2);
        double cruiseVelocity = Math.min(maxAcceleration * accelerationTime, maxDeceleration * decelerationTime);
        double cruiseDistance = distance - accelerationDistance - decelerationDistance;
        double cruiseTime = cruiseDistance / cruiseVelocity;

        double currentAccelerationDistance = maxAcceleration / 2 * Math.pow(Math.min(time, accelerationTime), 2);
        if (time < accelerationTime) // Acceleration time
            return new DualNum<>(currentAccelerationDistance, maxAcceleration * time, maxAcceleration);
        else if (time < accelerationTime + cruiseTime) // Cruise time
        {
            time -= accelerationTime;
            return new DualNum<>(currentAccelerationDistance + cruiseVelocity * time, cruiseVelocity, 0);
        }
        else if (time < accelerationTime + cruiseTime + decelerationTime) // Deceleration time
        {
            time -= accelerationTime;
            time -= cruiseTime;
            return new DualNum<>(currentAccelerationDistance + cruiseDistance + cruiseVelocity * time - maxDeceleration / 2 * Math.pow(time, 2), cruiseVelocity - maxDeceleration * time, -maxDeceleration);
        }
        else return new DualNum<>(distance, 0, 0); // Out of bounds

    }
}

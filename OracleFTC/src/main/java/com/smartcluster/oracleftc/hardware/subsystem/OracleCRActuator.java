package com.smartcluster.oracleftc.hardware.subsystem;

import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.hardware.servo.OracleCRServoImplEx;
import com.smartcluster.oracleftc.math.DualNum;
import com.smartcluster.oracleftc.math.Time;
import com.smartcluster.oracleftc.math.control.PIDController;
import com.smartcluster.oracleftc.math.control.TrapezoidalMotionProfile;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

public abstract class OracleCRActuator {
    private final Subsystem subsystem;
    private final String name;
    public PIDController pid;

    private double minimumVoltagePass;

    public TrapezoidalMotionProfile motionProfile;
    public double tolerance, integralIncrement;
    protected AtomicReference<Double> target = new AtomicReference<>(0.0);
    private final AtomicReference<Double> to = new AtomicReference<>(0.0), from=new AtomicReference<>(0.0);
    private final OracleCRServoImplEx[] crservos;

    private final ElapsedTime time = new ElapsedTime();
    public OracleCRActuator(Subsystem subsystem, String name, PIDController pid, TrapezoidalMotionProfile motionProfile, double tolerance, double minimumVoltagePass, double integralIncrement, OracleCRServoImplEx... motors)
    {
        this.subsystem=subsystem;
        this.name=name;
        this.pid=pid;
        this.motionProfile=motionProfile;
        this.crservos =motors;
        this.tolerance=tolerance;
        this.minimumVoltagePass=minimumVoltagePass;
        this.integralIncrement = integralIncrement;
    }
    /**
     * Sets the target of the actuator, the user needs to check for limits
     * @param target target of the actuator
     * @return if the operation succeeded
     */
    public abstract boolean setTarget(double target);

    public abstract DualNum<Time> getPosition();
    public final double getTarget()
    {
        return target.get();
    }

    public abstract Command reset();
    private boolean enabled=true;

    public final Command enable()
    {
        return new InstantCommand(()->enabled=true);
    }
    public final Command disable()
    {
        return new InstantCommand(()->{

            enabled=false;
            for (OracleCRServoImplEx motor : crservos) {
                motor.setPower(0.0);
            }
        });
    }

    public final void ManualSetFromPosition(double initialPosition)
    {
        from.set(initialPosition);
        time.reset();
    }

    public final Supplier<Boolean> isNotInMotion()
    {
        return () -> Math.abs(getPosition().get(0)-getTarget()) <= tolerance;
    }


    public final Command move(AtomicReference<Double> target)
    {
        return Command.builder()
                .init(()->
                {
                    from.set(getPosition().get(0));
                    time.reset();
                    setTarget(target.get());
                })
                .finished(isNotInMotion())
                .end((interrupted)->{
                    from.set(getPosition().get(0));
                    time.reset();
                })
                .requires(subsystem)
                .build();
    }

    // My silly attempt at making a dynamic minimum voltage regulator for when outside the tolerance
    private double integralInduced = 0, incrementalIntegral = 0;
    private long lastTimestamp=0;
    public double IntegralErrInduced(double distance)
    {
        //Add or remove a tiny value (values for ~30hz), fixing itself after overshooting
        incrementalIntegral += (distance > 0 ? 1 : -1)*integralIncrement;

        long timestamp = System.nanoTime();
        if (lastTimestamp != 0) {
            double deltaTime = (timestamp - lastTimestamp) / 1E9;
            integralInduced += deltaTime * distance;
            if (Double.isNaN(integralInduced)) integralInduced = 0;
        }
        lastTimestamp = timestamp;

        return integralInduced*incrementalIntegral;
    }

    public final Command update() {

        return Command.builder()
                .init(() -> {
                    from.set(getPosition().get(0));
                    to.set(getTarget());
                    time.reset();
                })
                .update(() -> {
                    if (to.get() != getTarget()) {
                        to.set(getTarget());
                    }

                    final double distance = to.get() - from.get();
                    DualNum<Time> mp = motionProfile.getMotionState(Math.abs(distance), time.seconds());
                    double power = pid.update(mp.get(0) * Math.signum(distance) + from.get(), getPosition().get(0));

                    // Servos have an minimum acceptance of voltage, too less of it and they can barely move without velocity.
//                    if (!isNotInMotion().get() && Math.abs(power) < minimumVoltagePass)
//                        power += integralInduced;
//                    else {
//                        // Reset the dynamic voltage regulator
//                        integralInduced = 0;
//                        incrementalIntegral = 0;
//                    }

                    for (OracleCRServoImplEx motor : crservos) {
                        if (enabled) motor.setPower(power);
                    }

//                    subsystem.telemetry.addData(String.format("%s.position", name), getPosition().get(0));
                    subsystem.telemetry.addData(String.format("%s.power", name), power);
                    subsystem.telemetry.addData(String.format("%s.integralInduced", name), integralInduced);
//                    subsystem.telemetry.addData(String.format("%s.target", name), getTarget());
//                    subsystem.telemetry.addData(String.format("%s.mp", name), mp.get(0));
                })
                .build();
    }
}

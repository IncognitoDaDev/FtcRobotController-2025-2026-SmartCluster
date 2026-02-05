package com.smartcluster.oracleftc.hardware.subsystem;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.hardware.wrappers.OmegaServoImplEx;
import com.smartcluster.oracleftc.math.control.TrapezoidalMotionProfile;

import java.util.concurrent.atomic.AtomicReference;

public abstract class OmegaServoActuator {
    private final Subsystem subsystem;
    private final String name;
    public final TrapezoidalMotionProfile motionProfile;
    protected AtomicReference<Double> target = new AtomicReference<>(0.0);
    private final ElapsedTime time = new ElapsedTime();
    private final AtomicReference<Double> to = new AtomicReference<>(0.0), from=new AtomicReference<>(0.0);

    private final OmegaServoImplEx[] servos;

    public OmegaServoActuator(Subsystem subsystem, String name, TrapezoidalMotionProfile motionProfile, OmegaServoImplEx... servos)
    {
        this.subsystem=subsystem;
        this.name=name;
        this.motionProfile=motionProfile;
        this.servos=servos;
    }

    public abstract Command reset();

    /**
     * Sets the target of the actuator , the user needs to check for limits
     * @param target target of the actuator
     * @return if the ope=ration succeeded
     */
    public abstract boolean setTarget(double target);
    public double getTarget()
    {
        return target.get();
    }

    public final Command move(AtomicReference<Double> target)
    {
        return Command.builder()
                .init(()->
                {
                    from.set(servos[0].getServo().getPosition());
                    time.reset();
                    setTarget(target.get());
                })
                .finished(()->Math.abs(servos[0].getServo().getPosition()-getTarget())<0.0001)
                .requires(subsystem)
                .build();
    }

    public final Command update()
    {
        return Command.builder()
                .init(()->{
                    from.set(servos[0].getServo().getPosition());
                    to.set(this.target.get());
                    time.reset();
                })
                .update(()->{
                    if(to.get()!=getTarget())
                    {
                        to.set(this.target.get());
                    }
                    final double distance = to.get()-from.get();
                    double position = motionProfile.getMotionState(Math.abs(distance), time.seconds()).get(0) * Math.signum(distance)+from.get();
                    for(OmegaServoImplEx servo: servos)
                    {
                        servo.setPosition(position);
                    }

//                    subsystem.telemetry.addData(String.format("%s.position", name), position);
//                    subsystem.telemetry.addData(String.format("%s.target", name), getTarget());

                })
                .build();
    }
}

package com.smartcluster.oracleftc.hardware.subsystem;

import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.hardware.OracleLynxVoltageSensor;
import com.smartcluster.oracleftc.math.DualNum;
import com.smartcluster.oracleftc.math.Time;
import com.smartcluster.oracleftc.math.control.PIDController;
import com.smartcluster.oracleftc.math.control.TrapezoidalMotionProfile;

import java.util.concurrent.atomic.AtomicReference;

public abstract class CRActuator {
    private final Subsystem subsystem;
    private final String name;
    public PIDController pid;
    public TrapezoidalMotionProfile motionProfile;
    public double tolerance;
    protected AtomicReference<Double> target = new AtomicReference<>(0.0);
    private final AtomicReference<Double> to = new AtomicReference<>(0.0), from=new AtomicReference<>(0.0);
    private final CRServoImplEx[] crservos;

    private final ElapsedTime time = new ElapsedTime();
    public CRActuator(Subsystem subsystem, String name, PIDController pid, TrapezoidalMotionProfile motionProfile, double tolerance, CRServoImplEx... motors)
    {
        this.subsystem=subsystem;
        this.name=name;
        this.pid=pid;
        this.motionProfile=motionProfile;
        this.crservos =motors;
        this.tolerance=tolerance;
    }
    /**
     * Sets the target of the actuator, the user needs to check for limits
     * @param target target of the actuator
     * @return if the operation succeeded
     */
    public abstract boolean setTarget(double target);
    public double getFeedforward()
    {
        return 0.0;
    }
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
            for (CRServoImpl motor :
                    crservos) {
                motor.setPower(0.0);
            }
        });
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
                .finished(()->Math.abs(getPosition().get(0)-getTarget())<tolerance)
                .end((interrupted)->{
                    from.set(getPosition().get(0));
                    time.reset();
                })
                .requires(subsystem)
                .build();
    }

    public final Command update()
    {

        return Command.builder()
                .init(()->{
                    from.set(getPosition().get(0));
                    to.set(getTarget());
                    time.reset();
                })
                .update(()->{
                    if(to.get()!=getTarget())
                    {
                        to.set(getTarget());
                    }
                    final double distance = to.get()-from.get();
                    DualNum<Time> mp = motionProfile.getMotionState(Math.abs(distance),
                            time.seconds());
                    double power = pid.update(mp.get(0) *Math.signum(distance)+from.get(),
                            getPosition().get(0))+getFeedforward();
                    for (CRServoImpl motor :
                            crservos) {
                        if(enabled) motor.setPower(power);
                    }
                    subsystem.telemetry.addData(String.format("%s.position", name), getPosition().get(0));
                    subsystem.telemetry.addData(String.format("%s.power", name), power);
                    subsystem.telemetry.addData(String.format("%s.target", name), getTarget());
                    subsystem.telemetry.addData(String.format("%s.mp", name), mp.get(1));
                })
                .build();
    }
}

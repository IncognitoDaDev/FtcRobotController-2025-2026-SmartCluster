package com.smartcluster.oracleftc.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

@SuppressWarnings("unused")
public class WaitCommand extends Command {

    private final long delay;
    private final ElapsedTime time = new ElapsedTime();
    private final Supplier<Boolean> supplier;

    public WaitCommand(long delay) {
        this.delay = delay;
        this.supplier = () -> false;
    }

    public WaitCommand(Supplier<Boolean> condition) {
        this.delay = -1;
        this.supplier = condition;
    }

    @Override
    public void init() {
        time.reset();
    }


    @Override
    public boolean finished() {

        if (delay > 0)
            return time.milliseconds() > delay;
        else return supplier.get();
    }


}

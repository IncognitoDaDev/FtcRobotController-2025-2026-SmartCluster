package com.smartcluster.oracleftc.commands;

import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;

import java.util.Set;

@SuppressWarnings("unused")
public class InstantCommand extends Command {
    private final Runnable runnable;
    private final Subsystem subsystem;
    public InstantCommand(Runnable runnable) {
        this.runnable = runnable;
        subsystem=null;
    }
    public InstantCommand(Runnable runnable, Subsystem subsystem) {
        this.runnable=runnable;
        this.subsystem=subsystem;
    }

    @Override
    public void init() {
        runnable.run();
    }

    @Override
    public boolean finished() {
        return true;
    }


    @Override
    public Set<Subsystem> requires() {
        if(subsystem==null) return null;
        return Set.of(subsystem);
    }
}

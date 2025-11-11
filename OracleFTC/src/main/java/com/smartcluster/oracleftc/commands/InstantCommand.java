package com.smartcluster.oracleftc.commands;

@SuppressWarnings("unused")
public class InstantCommand extends Command {
    private final Runnable runnable;

    public InstantCommand(Runnable runnable) {
        this.runnable = runnable;
    }

    @Override
    public void init() {
        runnable.run();
    }

    @Override
    public boolean finished() {
        return true;
    }
}

package com.smartcluster.oracleftc.commands;

import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;

import java.util.HashSet;
import java.util.Set;

@SuppressWarnings("unused")
public class RaceCommand extends Command {

    private final Command[] commands;

    public RaceCommand(Command... commands) {
        this.commands = commands;
    }

    @Override
    public void init() {
        for (Command command :
                commands) {
            command.init();
        }
    }

    @Override
    public void update() {
        for (Command command :
                commands) {
            command.update();
        }
    }

    public int finishedCommandIndex=-1;

    @Override
    public boolean finished() {
        if(finishedCommandIndex!=-1) return true;
        for (int i = 0; i < commands.length; i++) {
            if (commands[i].finished()) {
                finishedCommandIndex = i;
                return true;
            }
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        for (int i = 0; i < commands.length; i++) {
            if (i != finishedCommandIndex) {
                commands[i].end(true);
            } else commands[i].end(interrupted);
        }
    }
    @Override
    public Set<Subsystem> requires() {
        Set<Subsystem> subsystemsSet = new HashSet<>();
        for (Command command: commands) {
            Set<Subsystem> set = command.requires();
            if(set!=null)
                subsystemsSet.addAll(set);
        }
        return subsystemsSet;
    }
}

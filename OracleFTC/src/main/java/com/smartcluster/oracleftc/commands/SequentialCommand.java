package com.smartcluster.oracleftc.commands;

import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;

import java.util.Set;

@SuppressWarnings("unused")
public class SequentialCommand extends Command {
    private final Command[] commands;
    private int commandIndex;
    private boolean finished;

    public SequentialCommand(Command... commands) {
        this.commands = commands;
        commandIndex = 0;
        finished = false;
    }

    @Override
    public void init() {
        super.init();
        commandIndex = 0;
        finished = false;
        if (commands.length == 0) {
            finished = true;
            return;
        }
        commands[commandIndex].init();
    }

    @Override
    public void update() {
        super.update();
        if (commandIndex < commands.length) {
            commands[commandIndex].update();
            if (commands[commandIndex].finished()) {
                commands[commandIndex].end(false);
                commandIndex++;
                if (commandIndex < commands.length) commands[commandIndex].init();
            }
        } else finished = true;
    }

    @Override
    public boolean finished() {
        return finished;
    }

    @Override
    public boolean interruptable() {
        return commands[commandIndex].interruptable();
    }

    @Override
    public void end(boolean interrupted) {
        if (commandIndex < commands.length)
        {
            if (!interrupted)
                commands[commandIndex].end(false);
            else {
                commands[commandIndex].end(true);
            }
        }

    }

    @Override
    public Set<Subsystem> requires() {
        if(commandIndex==commands.length) return null;
        return commands[commandIndex].requires();
    }
}

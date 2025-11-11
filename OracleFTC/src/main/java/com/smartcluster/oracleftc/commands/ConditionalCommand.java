package com.smartcluster.oracleftc.commands;

import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;
import java.util.Set;
import java.util.function.Supplier;

@SuppressWarnings("unused")
public class ConditionalCommand extends Command {

    private final Command[] commands;
    private final Supplier<Integer> condition;
    private int commandIndex = 0;

    public ConditionalCommand(Supplier<Integer> condition, Command... commands) {
        this.commands = commands;
        this.condition = condition;
    }

    public ConditionalCommand(Supplier<Boolean> condition, Command ifCommand, Command elseCommand) {
        this(() -> (condition.get()) ? 0 : 1, ifCommand, elseCommand);
    }

    @Override
    public void init() {
        commandIndex = condition.get();
        if (commandIndex < commands.length)
            commands[commandIndex].init();
    }

    @Override
    public void update() {
        commands[commandIndex].update();
    }

    @Override
    public boolean finished() {
        return commands[commandIndex].finished();
    }

    @Override
    public void end(boolean interrupted) {
        commands[commandIndex].end(interrupted);
    }

    @Override
    public boolean interruptable() {
        return commands[commandIndex].interruptable();
    }

    @Override
    public Set<Subsystem> requires() {
        return commands[commandIndex].requires();
    }
}

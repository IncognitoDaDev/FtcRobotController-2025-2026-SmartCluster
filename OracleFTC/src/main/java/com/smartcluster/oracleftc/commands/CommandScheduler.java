package com.smartcluster.oracleftc.commands;

import java.util.ArrayList;
import java.util.List;

public class CommandScheduler {

    private final List<Command> commandsToSchedule = new ArrayList<>();
    private final List<Command> activeCommands = new ArrayList<>();
    /**
     * Clear a scheduled command and interrupts it if it is active
     * @param command target command
     */
    public synchronized void cancel(Command command) {


        commandsToSchedule.remove(command);
        if (activeCommands.remove(command)) {
            command.end(true);
        }
    }
    /**
     * Clear all scheduled command and interrupts them if they are active
     */
    public synchronized void cancelAll() {
        commandsToSchedule.clear();
        activeCommands.forEach(command -> command.end(true));
    }

    public boolean finished(Command command) {
        return !commandsToSchedule.contains(command) &&
                !activeCommands.contains(command);
    }

    public void schedule(Command command) {
        commandsToSchedule.add(command);
    }

    public void update() throws InterruptedException {
        for (Command command :
                commandsToSchedule) {
            command.init();
            activeCommands.add(command);
        }
        commandsToSchedule.clear();
        activeCommands.forEach(Command::update);
        activeCommands.removeIf(command -> {
            boolean finished = command.finished();
            if (finished) {
                command.end(false);
            }
            return finished;
        });

    }
}

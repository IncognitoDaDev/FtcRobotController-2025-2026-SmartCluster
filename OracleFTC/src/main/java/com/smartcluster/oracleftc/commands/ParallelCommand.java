package com.smartcluster.oracleftc.commands;

import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

@SuppressWarnings("unused")
public class ParallelCommand extends Command {

    private final Command[] commands;

    public ParallelCommand(Command... commands) {
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


    public boolean finished() {
        return Arrays.stream(commands).allMatch(Command::finished);
    }

    @Override
    public void end(boolean interrupted) {
        for (Command command :
                commands) {
            command.end(interrupted);
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

package com.smartcluster.oracleftc.commands;


import android.content.Context;

import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.util.RobotLog;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;

import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Class to represent a unit of code; the base of complex robot behaviour, allows splitting sequential
 * and parallel code into smaller and more manageable units of code to allow parallelism without threads.
 *
 * <p>
 * A command's lifetime is as follows:
 *     <ul>
 *         <li>The command is scheduled using {@link ThreadedCommandScheduler#schedule(Command)}</li>
 *         <li>
 *              <p>During each {@link ThreadedCommandScheduler#update()} the command advances in it's lifetime</p>
 *              <ul>
 *                  <li>First the command is initialized in {@linkplain Command#init()}</li>
 *                  <li>Then the command is updated with {@linkplain Command#update()} and checked
 *                  for its end with {@linkplain Command#finished()}</li>
 *                  <li>If the command is finished, then {@linkplain Command#end(boolean)} gets called
 *                  with false, to signify it didn't end prematurely(interrupted)</li>
 *                  <li>Alternatively, if the command is interrupted, {@linkplain Command#end(boolean)}
 *                  gets called with true</li>
 *              </ul>
 *         </li>
 *     </ul>
 * </p>
 *
 * <p>
 *     The true power of commands come when you group them, since command groups are {@linkplain Command} themselves.
 * </p>
 * <p>
 *     There are also additional features, such as {@linkplain Command#requires()}, which helps the
 *     {@linkplain ThreadedCommandScheduler} parallelize commands to improve loop times. Also, in case you
 *     don't want to extend {@linkplain Command}, you can use {@link CommandBuilder} to construct it
 *     from lambdas.
 * </p>
 *
 * @see SequentialCommand
 * @see ConditionalCommand
 * @see ParallelCommand
 * @see RaceCommand
 * @see WaitCommand
 * @see ThreadedCommandScheduler
 */
@SuppressWarnings({"unused"})
public abstract class Command {
    public static CommandBuilder builder() {
        return new CommandBuilder();
    }
    private static OpModeManagerImpl opModeManager;

    @OnCreateEventLoop
    public static void attachEventLoop(Context context, FtcEventLoop eventLoop) {
        opModeManager=eventLoop.getOpModeManager();
    }

    public static Command wrap(Supplier<Command> command)
    {
        AtomicReference<Command> generatedCommand = new AtomicReference<>();
        return Command.builder()
                .init(()->
                {
                    generatedCommand.set(command.get());
                    generatedCommand.get().init();
                })
                .update(()-> generatedCommand.get().update())
                .finished(()->
                    generatedCommand.get().finished()
                )
                .end((interrupted)-> generatedCommand.get().end(interrupted))

                .build();
    }

    public static void run(Command command) {
        command.init();
        while (!command.finished() && !Thread.currentThread().isInterrupted()) {
            command.update();
        }
        command.end(false);
    }
    public static void run(Command... commands) {
        for(Command command: commands)
            command.init();

        int commandsFinished = 0;
        boolean[] commandFinished = new boolean[commands.length];

        while (commandsFinished<commands.length)
        {
            for(int i=0;i<commands.length;i++)
            {
                if(!commandFinished[i])
                {
                    commands[i].update();
                    if(commands[i].finished())
                    {
                        commands[i].end(false);
                        commandFinished[i]=true;
                        commandsFinished++;
                    }
                }
            }
        }


    }

    public void init() {
    }

    public void update() {
    }

    public boolean finished() {
        return false;
    }

    public void end(boolean interrupted) {
    }

    public boolean interruptable() {
        return true;
    }

    public Set<Subsystem> requires() {
        return new HashSet<>();
    }

    public static class CommandBuilder {
        private Runnable init = () -> {
        };
        private Runnable update = () -> {
        };
        private Supplier<Boolean> finished = () -> false;
        private Consumer<Boolean> end = (finished) -> {
        };
        private Supplier<Boolean> interruptable = () -> true;
        private Supplier<Set<Subsystem>> requires = () -> null;

        public CommandBuilder init(Runnable init) {
            this.init = init;
            return this;
        }

        public CommandBuilder update(Runnable update) {
            this.update = update;
            return this;
        }

        public CommandBuilder finished(Supplier<Boolean> finished) {
            this.finished = finished;
            return this;
        }

        public CommandBuilder end(Consumer<Boolean> end) {
            this.end = end;
            return this;
        }

        public CommandBuilder interruptable(boolean interruptable) {
            this.interruptable = () -> interruptable;
            return this;
        }

        public CommandBuilder interruptable(Supplier<Boolean> interruptable) {
            this.interruptable = interruptable;
            return this;
        }

        public CommandBuilder requires(Subsystem subsystem) {
            this.requires = () -> {
                Set<Subsystem> set = new HashSet<>();
                set.add(subsystem);
                return set;
            };
            return this;
        }

        public CommandBuilder requires(Set<Subsystem> subsystems) {
            this.requires = () -> subsystems;
            return this;
        }

        public CommandBuilder requires(Supplier<Set<Subsystem>> subsystemsSupplier) {
            this.requires = subsystemsSupplier;
            return this;
        }

        public Command build() {
            return new Command() {
                @Override
                public void init() {
                    CommandBuilder.this.init.run();
                }

                @Override
                public void update() {
                    CommandBuilder.this.update.run();
                }

                @Override
                public boolean finished() {
                    return CommandBuilder.this.finished.get();
                }

                @Override
                public void end(boolean interrupted) {
                    CommandBuilder.this.end.accept(interrupted);
                }

                @Override
                public boolean interruptable() {
                    return CommandBuilder.this.interruptable.get();
                }

                @Override
                public Set<Subsystem> requires() {
                    return requires.get();
                }
            };
        }
    }
}

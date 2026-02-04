package com.smartcluster.oracleftc.fsm;


import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.CommandScheduler;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;

/**
 * <p>
 * A Finite State Machine, or FSM, is a computation model that can be used to simulate sequential
 * logic, or, in other words, to represent and control execution flow. The basic idea behind a FSM
 * is that you represent the robot's behaviour as a collection of states and transitions between states.
 * The transitions between the states are then triggered by different events(a button being pressed
 * on a gamepad or sensor logic)
 * </p>
 * <p>
 * More detailed, this FSM implementation associates with each state and transition a {@link Command},
 * which is executed during the state,
 * </p>
 *
 * @param <T> enum representing the possible states
 */
@SuppressWarnings({"unused"})
public class FSM<T extends Enum<T>> {

    private final CommandScheduler scheduler;
    private final Map<T, Command> stateCommands;
    private final Map<T, List<Transition<T>>> transitions;
    private T currentState;
    private Command currentCommand;
    private Transition<T> currentTransition;

    public FSM(T initialState, Map<T, Command> stateCommands,
               Map<T, List<Transition<T>>> transitions, CommandScheduler scheduler) {
        currentState = initialState;
        this.stateCommands = stateCommands;
        this.transitions = transitions;
        this.scheduler=scheduler;
        currentCommand = stateCommands.get(initialState);
        if (currentCommand != null) {
            scheduler.schedule(currentCommand);
        }
    }

    public static <T extends Enum<T>> FSMBuilder<T> builder() {
        return new FSMBuilder<>();
    }

    public void update()  {
        if (currentTransition != null) {
            if (scheduler.finished(currentTransition.getCommand())) {
                currentState = currentTransition.getTargetState();
                currentCommand = stateCommands.get(currentState);
                if (currentCommand != null) {
                    scheduler.schedule(currentCommand);
                }
                currentTransition = null;
            }
        } else {
            for (Transition<T> transition : Objects.requireNonNull(
                transitions.getOrDefault(currentState, new ArrayList<>()))) {
                if (transition.getCondition()) {
                    currentTransition = transition;
                    scheduler.schedule(currentTransition.getCommand());
                    scheduler.cancel(currentCommand);
                    currentCommand = null;
                    break;
                }
            }

        }
    }

    public T getCurrentState() {
        return currentState;
    }

    public Transition<T> getCurrentTransition() {
        return currentTransition;
    }

    public static class FSMBuilder<T extends Enum<T>> {
        private final Map<T, Command> stateCommands = new HashMap<>();
        private final Map<T, List<Transition<T>>> transitions = new HashMap<>();
        private T initialState;

        public FSMBuilder<T> initial(T state) {
            initialState = state;
            return this;
        }

        /**
         * Adds a {@link Command} to be executed during the state. The {@link Command} is scheduled
         * when entering the state and it is interrupted if a transition occurs before it finished.
         *
         * @param state   target state
         * @param command {@link Command} to be executed during the state
         * @return builder
         */
        public FSMBuilder<T> state(T state, Command command) {
            if (!command.interruptable()) {
                throw new IllegalArgumentException(
                    "FSM state commands are required to be interruptable");
            }

            stateCommands.put(state, command);
            return this;
        }

        /**
         * Adds a {@link Transition} between the two states. The transition's command is scheduled
         * by the trigger and the transition is finished when the transition command is finished. The
         * FSM then goes to the end state.
         *
         * @param from    start state
         * @param to      end state
         * @param trigger the trigger for the transition
         * @param command {@link Command} to be executed during the transition
         * @return builder
         */
        public FSMBuilder<T> transition(T from, T to, Supplier<Boolean> trigger, Command command) {
            List<Transition<T>> elements = transitions.getOrDefault(from, new ArrayList<>());
            Objects.requireNonNull(elements).add(new Transition<T>() {
                @Override
                public boolean getCondition() {
                    return trigger.get();
                }

                @Override
                public Command getCommand() {
                    return command;
                }

                @Override
                public T getTargetState() {
                    return to;
                }
            });
            transitions.put(from, elements);
            return this;
        }

        /**
         * Adds a {@link Transition} from the stating state. The transition's command is scheduled
         * by the trigger and the transition is finished when the transition command is finished.
         * <p>
         * When the transition ends, the target state is dictated by the supplier. This allows to
         * implement fail-saves or alternative transitions.
         * </p>
         *
         * @param from    start state
         * @param to      end state function
         * @param trigger the trigger for the transition
         * @param command {@link Command} to be executed during the transition
         * @return builder
         */
        public FSMBuilder<T> transition(T from, Supplier<T> to, Supplier<Boolean> trigger,
                                        Command command) {
            List<Transition<T>> elements = transitions.getOrDefault(from, new ArrayList<>());
            Objects.requireNonNull(elements).add(new Transition<T>() {
                @Override
                public boolean getCondition() {
                    return trigger.get();
                }

                @Override
                public Command getCommand() {
                    return command;
                }

                @Override
                public T getTargetState() {
                    return to.get();
                }
            });
            transitions.put(from, elements);
            return this;
        }

        /**
         * Builds the FSM
         * @return the FSM
         */
        public FSM<T> build(CommandScheduler scheduler) {
            return new FSM<T>(initialState, stateCommands, transitions, scheduler);
        }
    }
}

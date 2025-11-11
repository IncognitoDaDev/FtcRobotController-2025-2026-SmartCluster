package com.smartcluster.oracleftc.autonomous;

import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.ThreadedCommandScheduler;
import com.smartcluster.oracleftc.fsm.FSM;
import com.smartcluster.oracleftc.fsm.Transition;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;

public class AutonomousFSMBuilder<T extends Enum<T>> {
    private final Map<T, List<Transition<T>>> transitions = new HashMap<>();
    private T initialState;

    /**
     * Sets the initial autonomous state
     * @param state initial state
     * @return builder
     */
    public AutonomousFSMBuilder<T> initial(T state) {
        initialState = state;
        return this;
    }

    public AutonomousFSMBuilder<T> step(T from, T to, Command command)
    {
        List<Transition<T>> elements = transitions.getOrDefault(from, new ArrayList<>());
        Objects.requireNonNull(elements).add(new Transition<T>() {
            @Override
            public boolean getCondition() {
                return true;
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

    public AutonomousFSMBuilder<T> failsafe(T from, T to, Supplier<Boolean> condition, Command command)
    {
        List<Transition<T>> elements = transitions.getOrDefault(from, new ArrayList<>());
        Objects.requireNonNull(elements).add(new Transition<T>() {
            @Override
            public boolean getCondition() {
                return condition.get();
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

    public FSM<T> build(ThreadedCommandScheduler scheduler)
    {
        return new FSM<T>(initialState, new HashMap<>(), transitions,scheduler);
    }
}
